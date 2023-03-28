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

#include "blue_control/base_controller.hpp"

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace blue_control
{

BaseController::BaseController()
: Node("blue_control")
{
  // Create a publisher to publish the current desired RC values
  rc_override_publisher_ =
    this->create_publisher<mavros_msgs::msg::OverrideRCIn>("/mavros/rc/override", 1);

  // Start a timer to publish the current desired PWM values at a frequency of 50hz
  timer_ = create_wall_timer(20ms, std::bind(&Bridge::publishOverrideRCIn, this));

  // Set up a service to manage whether or not the RC values will be overridden
  enable_override_service_ = this->create_service<std_srvs::srv::SetBool>(
    "/blue_control/rc/override/enable",
    std::bind(&Bridge::enableOverride, this, std::placeholders::_1, std::placeholders::_2));
}

void Bridge::publishOverrideRCIn() const
{
  // Only publish the override values if the bridge has been enabled
  if (override_enabled_) {
    rc_override_publisher_->publish(current_pwm_values_);
  }
}

void Bridge::enableOverride(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,  // NOLINT
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)      // NOLINT
{
  // Enable/disable publishing the override messages
  override_enabled_ = request->data;

  // Set the response according to whether or not the update was done properly
  response->success = (override_enabled_ == request->data);
}

}  // namespace blue_control
