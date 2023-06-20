// Copyright 2023, Evan Palmer
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
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
#include <tf2_eigen/tf2_eigen.hpp>

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
  this->declare_parameter("use_battery_state", false);

  Eigen::VectorXd convergence_diag =
    convertVectorToEigenVector(this->get_parameter("convergence_rate").as_double_array());
  convergence_rate_ = convergence_diag.asDiagonal().toDenseMatrix();
  sliding_gain_ = this->get_parameter("sliding_gain").as_double();
  boundary_thickness_ = this->get_parameter("boundary_thickness").as_double();
  total_velocity_error_ = blue::dynamics::Vector6d::Zero();
  use_battery_state_ = this->get_parameter("use_battery_state").as_bool();

  // Update the reference signal when a new command is received
  cmd_sub_ = this->create_subscription<blue_msgs::msg::Reference>(
    "/blue/ismc/cmd", 1, [this](blue_msgs::msg::Reference::ConstSharedPtr msg) { cmd_ = *msg; });
}

mavros_msgs::msg::OverrideRCIn ISMC::update()
{
  blue::dynamics::Vector6d velocity;
  tf2::fromMsg(odom_.twist.twist, velocity);

  // Calculate the velocity error
  blue::dynamics::Vector6d velocity_error;
  tf2::fromMsg(cmd_.twist, velocity_error);
  velocity_error -= velocity;

  // There is no suitable tf2_eigen function for Accel types :(
  blue::dynamics::Vector6d desired_accel;  // NOLINT
  desired_accel << cmd_.accel.linear.x, cmd_.accel.linear.y, cmd_.accel.linear.z,
    cmd_.accel.angular.x, cmd_.accel.angular.y, cmd_.accel.angular.z;

  // Get the current rotation of the vehicle in the inertial frame
  Eigen::Quaterniond orientation;
  tf2::fromMsg(odom_.pose.pose.orientation, orientation);

  // Make sure to update the velocity error integral term BEFORE calculating the sliding surface
  // (the integral is up to time "t")
  total_velocity_error_ += velocity_error * dt_;

  // Calculate the sliding surface
  blue::dynamics::Vector6d surface =
    velocity_error + convergence_rate_ * total_velocity_error_;  // NOLINT

  // Apply the sign function to the surface
  surface.unaryExpr([this](double x) { return tanh(x / boundary_thickness_); });

  const blue::dynamics::Vector6d forces =
    hydrodynamics_.inertia.getInertia() * (desired_accel + convergence_rate_ * velocity_error) +
    hydrodynamics_.coriolis.calculateCoriolis(velocity) * velocity +
    hydrodynamics_.damping.calculateDamping(velocity) * velocity +
    hydrodynamics_.restoring_forces.calculateRestoringForces(orientation.toRotationMatrix()) +
    sliding_gain_ * surface;

  // Multiply the desired forces by the pseudoinverse of the thruster configuration matrix
  // The size of this vector will depend on the number of thrusters so we don't assign it to a
  // fixed-size matrix
  const Eigen::VectorXd pwms = tcm_.completeOrthogonalDecomposition().pseudoInverse() * forces;

  // Convert the thruster forces to PWM values
  if (use_battery_state_) {
    pwms.unaryExpr([this](double x) {
      return blue::dynamics::calculatePwmFromThrustSurface(x, battery_state_.voltage);
    });
  } else {
    pwms.unaryExpr([this](double x) { return blue::dynamics::calculatePwmFromThrustCurve(x); });
  }

  mavros_msgs::msg::OverrideRCIn msg;

  // We only modify the first "n" channels where "n" is the total number of thrusters
  for (uint16_t & channel : msg.channels) {
    channel = mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE;
  }

  // Calculate the deadzone band
  std::tuple<int, int> deadband;

  if (use_battery_state_) {
    deadband = blue::dynamics::calculateDeadZone(battery_state_.voltage);
  } else {
    deadband = blue::dynamics::calculateDeadZone();
  }

  // Set the PWM values
  for (uint16_t i = 0; i < pwms.size(); i++) {
    auto pwm = static_cast<uint16_t>(pwms[i]);

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
