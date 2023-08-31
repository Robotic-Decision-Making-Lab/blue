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

#include "blue_control/controller.hpp"

#include <memory>

#include "blue_utils/eigen.hpp"
#include "blue_utils/tf2.hpp"

namespace blue::control
{

Controller::Controller(const std::string & node_name)
: Node(node_name)
{
  // Declare ROS parameters
  this->declare_parameter("mass", 13.5);
  this->declare_parameter("buoyancy", 112.80);
  this->declare_parameter("weight", 114.80);
  this->declare_parameter("center_of_gravity", std::vector<double>({0.0, 0.0, 0.0}));
  this->declare_parameter("center_of_buoyancy", std::vector<double>({0.0, 0.0, 0.0}));
  this->declare_parameter("ocean_current", std::vector<double>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
  this->declare_parameter("num_thrusters", 8);
  this->declare_parameter("control_rate", 200.0);
  this->declare_parameter("inertia_tensor_coeff", std::vector<double>({0.16, 0.16, 0.16}));
  this->declare_parameter(
    "added_mass_coeff", std::vector<double>({-5.50, -12.70, -14.60, -0.12, -0.12, -0.12}));
  this->declare_parameter(
    "linear_damping_coeff", std::vector<double>({-4.03, -6.22, -5.18, -0.07, -0.07, -0.07}));
  this->declare_parameter(
    "quadratic_damping_coeff", std::vector<double>({-18.18, -21.66, -36.99, -1.55, -1.55, -1.55}));
  this->declare_parameter(
    "frame", std::vector<bool>({true, true, false, false, true, false, false, true}));

  // clang-format off
  this->declare_parameter(
    "tcm", std::vector<double>({-0.707,  -0.707,   0.707,  0.707,    0.0,    0.0,   0.0,   0.0,
                                 0.707,  -0.707,   0.707, -0.707,    0.0,    0.0,   0.0,   0.0,
                                   0.0,     0.0,     0.0,    0.0,    1.0,   -1.0,  -1.0,   1.0,
                                   0.0,     0.0,     0.0,    0.0, -0.218, -0.218, 0.218, 0.218,
                                   0.0,     0.0,     0.0,    0.0,  -0.12,   0.12, -0.12,  0.12,
                                0.1888, -0.1888, -0.1888, 0.1888,    0.0,    0.0,   0.0,   0.0}));
  // clang-format on

  // Get hydrodynamic parameters
  const double mass = this->get_parameter("mass").as_double();
  const double buoyancy = this->get_parameter("buoyancy").as_double();
  const double weight = this->get_parameter("weight").as_double();
  const Eigen::Vector3d inertia_tensor_coeff = blue::utility::vectorToEigen<double>(
    this->get_parameter("inertia_tensor_coeff").as_double_array(), 3, 1);
  const Eigen::Matrix<double, 6, 1> added_mass_coeff = blue::utility::vectorToEigen<double>(
    this->get_parameter("added_mass_coeff").as_double_array(), 6, 1);
  const Eigen::Matrix<double, 6, 1> linear_damping_coeff = blue::utility::vectorToEigen<double>(
    this->get_parameter("linear_damping_coeff").as_double_array(), 6, 1);
  const Eigen::Matrix<double, 6, 1> quadratic_damping_coeff = blue::utility::vectorToEigen<double>(
    this->get_parameter("quadratic_damping_coeff").as_double_array(), 6, 1);
  const Eigen::Vector3d center_of_gravity = blue::utility::vectorToEigen<double>(
    this->get_parameter("center_of_gravity").as_double_array(), 3, 1);
  const Eigen::Vector3d center_of_buoyancy = blue::utility::vectorToEigen<double>(
    this->get_parameter("center_of_buoyancy").as_double_array(), 3, 1);
  const Eigen::Matrix<double, 6, 1> ocean_current = blue::utility::vectorToEigen<double>(
    this->get_parameter("ocean_current").as_double_array(), 6, 1);

  // Get the thruster configuration matrix
  std::vector<double> tcm_vec = this->get_parameter("tcm").as_double_array();
  const int num_thrusters = this->get_parameter("num_thrusters").as_int();
  tcm_ = blue::utility::vectorToEigen<double, Eigen::RowMajor>(
    tcm_vec, static_cast<int>(tcm_vec.size() / num_thrusters), num_thrusters);

  // Initialize the hydrodynamic parameters
  hydrodynamics_ = blue::dynamics::HydrodynamicParameters(
    blue::dynamics::Inertia(mass, inertia_tensor_coeff, added_mass_coeff),
    blue::dynamics::Coriolis(mass, inertia_tensor_coeff, added_mass_coeff),
    blue::dynamics::Damping(linear_damping_coeff, quadratic_damping_coeff),
    blue::dynamics::RestoringForces(weight, buoyancy, center_of_buoyancy, center_of_gravity),
    blue::dynamics::CurrentEffects(ocean_current));

  // Setup the ROS things
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  accel_pub_ = this->create_publisher<geometry_msgs::msg::AccelStamped>(
    "/blue/state/accel", rclcpp::SensorDataQoS());
  rc_override_pub_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>(
    "mavros/rc/override", rclcpp::SystemDefaultsQoS());

  // clang-tidy and ROS conflict when creating subscriptions with ConstSharedPtr
  // NOLINTBEGIN(performance-unnecessary-value-param)
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/blue/local_position/odom", rclcpp::SensorDataQoS(),
    [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) -> void { updateOdomCb(msg); });

  arm_srv_ = this->create_service<std_srvs::srv::SetBool>(
    "blue/cmd/arm",
    [this](
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) -> void {
      armControllerCb(request, response);
    });
  // NOLINTEND(performance-unnecessary-value-param)

  // Convert the control loop frequency to seconds
  dt_ = 1 / this->get_parameter("control_rate").as_double();

  // Give the control loop its own callback group to avoid issues with long callbacks in the
  // default callback group
  control_loop_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(dt_),
    [this]() -> void {
      if (armed_) {
        rc_override_pub_->publish(calculateControlInput());
      }
    },
    control_loop_cb_group_);
}

void Controller::armControllerCb(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,  // NOLINT
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)      // NOLINT
{
  if (request->data) {
    // Run the controller arming function prior to actually arming the controller
    // This makes sure that any processing that needs to happen before the controller is *actually*
    // armed can occur
    onArm();

    // Arm the controller
    armed_ = true;
    response->success = true;
    response->message = "Controller armed.";
    RCLCPP_WARN(this->get_logger(), "Custom BlueROV2 controller armed.");
  } else {
    // Disarm the controller
    armed_ = false;
    response->success = true;
    response->message = "Controller disarmed.";
    RCLCPP_WARN(this->get_logger(), "Custom BlueROV2 controller disarmed.");

    // Run the controller disarming function after the controller has been fully disarmed
    onDisarm();
  }
}

void Controller::updateOdomCb(nav_msgs::msg::Odometry::ConstSharedPtr msg)  // NOLINT
{
  // Get the duration between the readings
  rclcpp::Time prev_stamp(odom_.header.stamp.sec, odom_.header.stamp.nanosec);
  rclcpp::Time current_stamp(msg->header.stamp.sec, msg->header.stamp.nanosec);
  const double dt = (current_stamp - prev_stamp).seconds();

  // Calculate the current acceleration using finite differencing and publish it for debugging
  geometry_msgs::msg::Accel accel;
  accel.linear.x = (msg->twist.twist.linear.x - odom_.twist.twist.linear.x) / dt;
  accel.linear.y = (msg->twist.twist.linear.y - odom_.twist.twist.linear.y) / dt;
  accel.linear.z = (msg->twist.twist.linear.z - odom_.twist.twist.linear.z) / dt;
  accel.angular.x = (msg->twist.twist.angular.x - odom_.twist.twist.angular.x) / dt;
  accel.angular.y = (msg->twist.twist.angular.y - odom_.twist.twist.angular.y) / dt;
  accel.angular.z = (msg->twist.twist.angular.z - odom_.twist.twist.angular.z) / dt;

  accel_ = accel;

  geometry_msgs::msg::AccelStamped accel_stamped;
  accel_stamped.header.frame_id = blue::transforms::kBaseLinkFrameId;
  accel_stamped.header.stamp = this->get_clock()->now();
  accel_stamped.accel = accel_;

  accel_pub_->publish(accel_stamped);

  // Publish the current transformation from the map frame to the base_link frame
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = "map";
  tf.header.stamp = this->get_clock()->now();
  tf.child_frame_id = "base_link";

  tf.transform.translation.x = msg->pose.pose.position.x;
  tf.transform.translation.y = msg->pose.pose.position.y;
  tf.transform.translation.z = msg->pose.pose.position.z;

  tf.transform.rotation.x = msg->pose.pose.orientation.x;
  tf.transform.rotation.y = msg->pose.pose.orientation.y;
  tf.transform.rotation.z = msg->pose.pose.orientation.z;
  tf.transform.rotation.w = msg->pose.pose.orientation.w;

  tf_broadcaster_->sendTransform(tf);

  // Update the current Odometry reading
  odom_ = *msg;
}

}  // namespace blue::control
