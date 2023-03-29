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
: Node(node_name, options),
  control_loop_freq_(250)
{
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.name = "control_loop_freq";
    desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    desc.description = "The frequency at which the control loop should run";
    desc.read_only = true;
    control_loop_freq_ = declare_parameter(desc.name, control_loop_freq_, desc);
  }

  // Create a publisher to publish the current desired RC values
  rc_override_pub_ =
    this->create_publisher<mavros_msgs::msg::OverrideRCIn>("/mavros/rc/override", 1);

  // If the control loop frequency is greater than 50hz, publish PWM values at the same rate as the
  // control loop, otherwise publish PWM values at 50hz.
  const double pwm_freq_sec = control_loop_freq_ > 50 ? (1 / control_loop_freq_) : 0.02;

  // Start a timer to publish the current desired PWM values at the
  timer_ = create_wall_timer(
    std::chrono::duration<double>(pwm_freq_sec), std::bind(&Controller::publishRcCb, this));

  // Set up a service to manage whether or not the RC values will be overridden
  enable_override_service_ = this->create_service<std_srvs::srv::SetBool>(
    "/blue_control/rc/override/enable",
    std::bind(&Controller::enableOverrideCb, this, std::placeholders::_1, std::placeholders::_2));
}

void Controller::setControlSignal(const mavros_msgs::msg::OverrideRCIn & control_input)
{
  control_signal_ = control_input;
}

void Controller::publishRcCb() const
{
  // Only publish the override values if the bridge has been enabled and the subscription
  // has been set up
  if (override_enabled_ && (rc_override_pub_->get_subscription_count() > 0)) {
    rc_override_pub_->publish(control_signal_);
  }
}

void Controller::enableOverrideCb(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,  // NOLINT
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)      // NOLINT
{
  // Enable/disable publishing the override messages
  override_enabled_ = request->data;

  // Set the response according to whether or not the update was done properly
  response->success = (override_enabled_ == request->data);
}

void Controller::setOdomPoseCb(const geometry_msgs::msg::PoseStamped & pose)
{
  // TODO(evan-palmer): update transforms here
  odom_pose_ = pose;
}

void Controller::proxySlamPoseCb(const geometry_msgs::msg::PoseStamped & pose)
{
  // TODO(evan-palmer): update to use correct coordinate frames here
  ext_nav_pub_->publish(pose);
}

}  // namespace blue::control
