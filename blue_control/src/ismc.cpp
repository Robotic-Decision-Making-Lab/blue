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

#include "blue_dynamics/thruster_dynamics.hpp"
#include "blue_utils/utils.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace blue::control
{

ISMC::ISMC()
: Controller("ismc"),
  initial_velocity_error_(blue::dynamics::Vector6d::Zero()),
  initial_acceleration_error_(blue::dynamics::Vector6d::Zero()),
  total_velocity_error_(blue::dynamics::Vector6d::Zero())
{
  // Declare the ROS parameters specific to this controller
  this->declare_parameter("integral_gain", std::vector<double>({1.0, 1.0, 1.0, 1.0, 1.0, 1.0}));
  this->declare_parameter("proportional_gain", std::vector<double>({1.0, 1.0, 1.0, 1.0, 1.0, 1.0}));
  this->declare_parameter("derivative_gain", std::vector<double>({1.0, 1.0, 1.0, 1.0, 1.0, 1.0}));
  this->declare_parameter("sliding_gain", 20.0);
  this->declare_parameter("boundary_thickness", 100.0);
  this->declare_parameter("use_battery_state", false);

  // Get the gain matrices
  Eigen::VectorXd integral_gain_coeff = blue::utils::convertVectorToEigenMatrix<double>(
    this->get_parameter("integral_gain").as_double_array(), 6, 1);
  Eigen::VectorXd proportional_gain_coeff = blue::utils::convertVectorToEigenMatrix<double>(
    this->get_parameter("proportional_gain").as_double_array(), 6, 1);
  Eigen::VectorXd derivative_gain_coeff = blue::utils::convertVectorToEigenMatrix<double>(
    this->get_parameter("derivative_gain").as_double_array(), 6, 1);

  integral_gain_ = integral_gain_coeff.asDiagonal().toDenseMatrix();
  proportional_gain_ = proportional_gain_coeff.asDiagonal().toDenseMatrix();
  derivative_gain_ = derivative_gain_coeff.asDiagonal().toDenseMatrix();
  sliding_gain_ = this->get_parameter("sliding_gain").as_double();
  boundary_thickness_ = this->get_parameter("boundary_thickness").as_double();
  use_battery_state_ = this->get_parameter("use_battery_state").as_bool();

  // Publish the desired wrench and errors to help with tuning and visualization
  desired_wrench_pub_ =
    this->create_publisher<geometry_msgs::msg::WrenchStamped>("/blue/ismc/desired_wrench", 1);
  velocity_error_pub_ =
    this->create_publisher<geometry_msgs::msg::TwistStamped>("/blue/ismc/velocity_error", 1);

  // Update the reference signal when a new command is received
  cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/blue/ismc/cmd_vel", 1,
    [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) { cmd_ = *msg; });
}

void ISMC::onArm()
{
  // Reset the total velocity error
  total_velocity_error_ = blue::dynamics::Vector6d::Zero();

  // Reset the initial conditions
  initial_velocity_error_ = blue::dynamics::Vector6d::Zero();
  initial_acceleration_error_ = blue::dynamics::Vector6d::Zero();

  // We need to calculate the initial conditions for the controller now. This includes the
  // initial velocity and acceleration errors

  // Start by calculating the velocity error i.c.
  blue::dynamics::Vector6d velocity;
  tf2::fromMsg(odom_.twist.twist, velocity);

  tf2::fromMsg(cmd_, initial_velocity_error_);
  initial_velocity_error_ -= velocity;

  // Now calculate the acceleration error i.c.
  // Assume that the desired acceleration is 0
  blue::dynamics::Vector6d accel;
  blue::utils::fromMsg(accel_, accel);
  initial_acceleration_error_ -= accel;
};

void ISMC::onDisarm()
{
  // Reset the total velocity error on disarm just to be safe
  total_velocity_error_ = blue::dynamics::Vector6d::Zero();

  // Reset the initial conditions too
  initial_velocity_error_ = blue::dynamics::Vector6d::Zero();
  initial_acceleration_error_ = blue::dynamics::Vector6d::Zero();
};

mavros_msgs::msg::OverrideRCIn ISMC::calculateControlInput()
{
  blue::dynamics::Vector6d velocity;
  tf2::fromMsg(odom_.twist.twist, velocity);

  // Calculate the velocity error
  blue::dynamics::Vector6d velocity_error;
  tf2::fromMsg(cmd_, velocity_error);
  velocity_error -= velocity;

  // Calculate the acceleration error; assume that the desired acceleration is 0
  blue::dynamics::Vector6d accel_error;
  blue::utils::fromMsg(accel_, accel_error);
  accel_error *= -1;

  // Publish the velocity error to help with debugging
  geometry_msgs::msg::TwistStamped velocity_error_msg;
  velocity_error_msg.header.frame_id = blue::transforms::kBaseLinkFrameId;
  velocity_error_msg.header.stamp = this->get_clock()->now();
  velocity_error_msg.twist = tf2::toMsg(velocity_error);
  velocity_error_pub_->publish(velocity_error_msg);

  // Get the current rotation of the vehicle in the inertial (map) frame
  // This is effectively the transformation of the orientation to the body (base_link) frame
  Eigen::Quaterniond orientation;
  tf2::fromMsg(odom_.pose.pose.orientation, orientation);

  // Make sure to update the velocity error integral term BEFORE calculating the sliding surface
  // (the integral is up to time "t")
  total_velocity_error_ += velocity_error * dt_;

  // Calculate the sliding surface
  blue::dynamics::Vector6d surface =
    proportional_gain_ * velocity_error + integral_gain_ * total_velocity_error_ +
    derivative_gain_ * accel_error - proportional_gain_ * initial_velocity_error_ -
    initial_acceleration_error_;

  // Apply the sign function to the surface
  // We use the tanh function to help reduce some of the chatter
  surface = surface.unaryExpr([this](double x) { return tanh(x / boundary_thickness_); });

  // Calculate the computed torque control
  blue::dynamics::Vector6d tau0 =
    hydrodynamics_.inertia.getInertia() *
      (proportional_gain_ * velocity_error + integral_gain_ * total_velocity_error_ +
       derivative_gain_ * accel_error) +
    (hydrodynamics_.coriolis.calculateCoriolis(velocity) +
     hydrodynamics_.damping.calculateDamping(velocity)) *
      velocity +
    hydrodynamics_.restoring_forces.calculateRestoringForces(orientation.toRotationMatrix());

  // Calculate the disturbance rejection torque
  blue::dynamics::Vector6d tau1 = -sliding_gain_ * surface;

  blue::dynamics::Vector6d forces = tau0 + tau1;

  // Publish the desired wrench for debugging purposes
  geometry_msgs::msg::WrenchStamped wrench;
  wrench.header.frame_id = blue::transforms::kBaseLinkFrameId;
  wrench.header.stamp = this->get_clock()->now();
  wrench.wrench = blue::utils::toMsg(forces);
  desired_wrench_pub_->publish(wrench);

  // Initialize an OverrideRCIn message with no change for the PWM values
  mavros_msgs::msg::OverrideRCIn msg;

  // Set all channels to "NOCHANGE" by default
  for (uint16_t & channel : msg.channels) {
    channel = mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE;
  }

  // The torques have been calculated for the base_link frame, but we need to transform them
  // to the base_link_frd frame for ArduSub.
  geometry_msgs::msg::TransformStamped transform;

  try {
    transform = tf_buffer_->lookupTransform(
      blue::transforms::kBaseLinkFrdFrameId, blue::transforms::kBaseLinkFrameId,
      tf2::TimePointZero);
  }
  catch (const tf2::TransformException & e) {
    RCLCPP_INFO(  // NOLINT
      this->get_logger(), "Could not transform from %s to %s: %s",
      blue::transforms::kBaseLinkFrameId.c_str(), blue::transforms::kBaseLinkFrdFrameId.c_str(),
      e.what());
    return msg;
  }

  geometry_msgs::msg::WrenchStamped wrench_frd;
  tf2::doTransform(wrench, wrench_frd, transform);
  blue::utils::fromMsg(wrench_frd.wrench, forces);

  // Multiply the desired forces by the pseudoinverse of the thruster configuration matrix
  // The size of this vector will depend on the number of thrusters so we don't assign it to a
  // fixed-size matrix
  Eigen::VectorXd thruster_forces = tcm_.completeOrthogonalDecomposition().pseudoInverse() * forces;

  // Convert the thruster forces to PWM values
  Eigen::VectorXi pwms;

  if (use_battery_state_) {
    pwms = thruster_forces.unaryExpr([this](double x) {
      return blue::dynamics::calculatePwmFromThrustSurface(x, battery_state_.voltage);
    });
  } else {
    pwms = thruster_forces.unaryExpr(
      [this](double x) { return blue::dynamics::calculatePwmFromThrustCurve(x); });
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
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
