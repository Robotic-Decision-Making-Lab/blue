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

#include "blue_control/controller.hpp"

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace blue::control
{

Controller::Controller(const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  double control_loop_freq = 250;
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.name = "control_loop_freq";
    desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    desc.description =
      "The frequency at which the control loop should run. This must be greater than 50Hz to "
      "override the RC inputs.";
    desc.read_only = true;
    control_loop_freq = declare_parameter(desc.name, control_loop_freq, desc);
  }

  rc_override_pub_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>(
    "/blue_bridge/rc/override", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
    [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr pose) -> void {
      updatePoseCb(std::move(pose));
    });

  start_control_ = this->create_service<std_srvs::srv::SetBool>(
    "/blue/control/run",
    [this](
      const std::shared_ptr<std_srvs::srv::SetBool::Request> & request,
      const std::shared_ptr<std_srvs::srv::SetBool::Response> & response) -> void {
      enableControlCb(request, response);
    });

  timer_ = create_wall_timer(
    std::chrono::duration<double>(1 / control_loop_freq), [this]() -> void { runControlLoopCb(); });

  // Set the initial pose time
  pose_time_ = this->get_clock()->now();
}

void Controller::runControlLoopCb()
{
  if (running_) {
    rc_override_pub_->publish(update());
  }
}

void Controller::enableControlCb(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> & request,
  const std::shared_ptr<std_srvs::srv::SetBool::Response> & response)
{
  running_ = request->data;

  // Set the response according to whether or not the update was done properly
  response->success = (running_ == request->data);
}

void Controller::updatePoseCb(geometry_msgs::msg::PoseStamped::ConstSharedPtr pose)  // NOLINT
{
  // Update the linear velocity
  double duration = rclcpp::Duration(pose->header.stamp - pose_time_).seconds();  // NOLINT

  odom_.twist.twist.linear.x = (pose->pose.position.x - odom_.pose.pose.position.x) / duration;
  odom_.twist.twist.linear.y = (pose->pose.position.y - odom_.pose.pose.position.y) / duration;
  odom_.twist.twist.linear.z = (pose->pose.position.z - odom_.pose.pose.position.z) / duration;

  // Now we can update the pose
  odom_.pose.pose = pose->pose;

  // Save the timestamp for the next measurement
  pose_time_ = pose->header.stamp;

  // TODO(evan-palmer): update transforms here
}

void Controller::updateAngularVelCb(sensor_msgs::msg::Imu::ConstSharedPtr imu)  // NOLINT
{
  // TODO(evan-palmer): update transforms here (if necessary)
  odom_.twist.twist.angular = imu->angular_velocity;
}

}  // namespace blue::control
