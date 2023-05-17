// Copyright 2023, Evan Palmer
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "blue_control/ismc.hpp"

#include <algorithm>
#include <cmath>

#include "blue_dynamics/thruster_dynamics.hpp"

namespace blue::control
{

ISMC::ISMC()
: Controller("ismc")
{
  // Declare the ROS parameters specific to this controller
  // There are additional parameters defined by the base controller as well
  this->declare_parameter(
    "convergence_rate", std::vector<double>({100.0, 100.0, 100.0, 100.0, 100.0, 100.0}));
  this->declare_parameter("sliding_gain", 0.0);
  this->declare_parameter("boundary_thickness", 0.0);

  Eigen::VectorXd convergence_diag =
    convertVectorToEigenVector(this->get_parameter("convergence_rate").as_double_array());
  convergence_rate_ = convergence_diag.asDiagonal().toDenseMatrix();
  sliding_gain_ = this->get_parameter("sliding_gain").as_double();
  boundary_thickness_ = this->get_parameter("boundary_thickness").as_double();
  total_velocity_error_ = Eigen::VectorXd::Zero(6);

  // Update the reference signal when a new command is received
  cmd_sub_ = this->create_subscription<blue_msgs::msg::Reference>(
    "/blue/ismc/cmd", 1, [this](blue_msgs::msg::Reference::ConstSharedPtr msg) { cmd_ = *msg; });
}

mavros_msgs::msg::OverrideRCIn ISMC::update()
{
  Eigen::VectorXd velocity(6);  // NOLINT
  velocity << odom_.twist.twist.linear.x, odom_.twist.twist.linear.y, odom_.twist.twist.linear.z,
    odom_.twist.twist.angular.x, odom_.twist.twist.angular.y, odom_.twist.twist.angular.z;

  // Calculate the velocity error
  Eigen::VectorXd velocity_error(6);
  velocity_error << cmd_.twist.linear.x, cmd_.twist.linear.y, cmd_.twist.linear.z,
    cmd_.twist.angular.x, cmd_.twist.angular.y, cmd_.twist.angular.z;
  velocity_error -= velocity;

  Eigen::VectorXd desired_accel(6);  // NOLINT
  desired_accel << cmd_.accel.linear.x, cmd_.accel.linear.y, cmd_.accel.linear.z,
    cmd_.accel.angular.x, cmd_.accel.angular.y, cmd_.accel.angular.z;

  // Get the current rotation of the vehicle in the inertial frame
  Eigen::Quaterniond orientation = Eigen::Quaterniond(
    odom_.pose.pose.orientation.w, odom_.pose.pose.orientation.x, odom_.pose.pose.orientation.y,
    odom_.pose.pose.orientation.z);

  // Make sure to update the velocity error integral term BEFORE calculating the sliding surface
  // (the integral is up to time "t")
  total_velocity_error_ += velocity_error * dt_;

  // Calculate the sliding surface
  Eigen::VectorXd surface = velocity_error + convergence_rate_ * total_velocity_error_;  // NOLINT

  // Apply the sign function to the surface
  surface.unaryExpr([this](double x) { return tanh(x / boundary_thickness_); });

  const Eigen::VectorXd forces =
    hydrodynamics_.inertia.getInertia() * (desired_accel + convergence_rate_ * velocity_error) +
    hydrodynamics_.coriolis.calculateCoriolis(velocity) * velocity +
    hydrodynamics_.damping.calculateDamping(velocity) * velocity +
    hydrodynamics_.restoring_forces.calculateRestoringForces(orientation.toRotationMatrix()) +
    sliding_gain_ * surface;

  // Multiply the desired forces by the pseudoinverse of the thruster configuration matrix
  const Eigen::VectorXd pwms = tcm_.completeOrthogonalDecomposition().pseudoInverse() * forces;

  // Convert the thruster forces to PWM values
  pwms.unaryExpr([this](double x) {
    return blue::dynamics::calculatePwmFromThrustSurface(x, battery_state_.voltage);
  });

  mavros_msgs::msg::OverrideRCIn msg;

  // We only modify the first "n" channels where "n" is the total number of thrusters
  for (uint16_t & channel : msg.channels) {
    channel = mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE;
  }

  const std::tuple<int, int> deadband = blue::dynamics::calculateDeadZone(battery_state_.voltage);

  for (uint16_t i = 0; i < pwms.size(); i++) {
    uint16_t pwm = static_cast<uint16_t>(pwms[i]);

    // Apply the deadband to the PWM values
    if (pwm > std::get<0>(deadband) && pwm < std::get<1>(deadband)) {
      pwm = blue::dynamics::kNoThrustPwm;
    }

    // Clamp the PWM to the valid PWM range
    msg.channels[i] = std::clamp(pwm, blue::dynamics::kMinPwm, blue::dynamics::kMaxPwm);
  }

  return msg;
}

}  // namespace blue::control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<blue::control::ISMC>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
