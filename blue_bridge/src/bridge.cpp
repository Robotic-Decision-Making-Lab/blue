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

#include "blue_bridge/bridge.hpp"

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace blue_bridge
{

Bridge::Bridge()
: Node("blue_bridge"),
  bridge_running_(false)
{
  // Create a subscription to get the desired PWM values
  rc_override_subscription_ = this->create_subscription<mavros_msgs::msg::OverrideRCIn>(
    "/blue/rc/override", 1,
    std::bind(&Bridge::updateCurrentPwmValues, this, std::placeholders::_1));

  // Create a publisher to publish the current desired RC values
  rc_override_publisher_ =
    this->create_publisher<mavros_msgs::msg::OverrideRCIn>("/mavros/rc/override", 1);

  // Start a timer to publish the current desired PWM values at a frequency of 50hz
  timer_ = create_wall_timer(20ms, std::bind(&Bridge::publishOverrideRCIn, this));
}

bool Bridge::active() const { return bridge_running_; }

void Bridge::publishOverrideRCIn() const
{
  // Only publish the override values if the bridge has been enabled
  if (active()) {
    rc_override_publisher_->publish(current_pwm_values_);
  }
}

void Bridge::updateCurrentPwmValues(const mavros_msgs::msg::OverrideRCIn & desired_pwm_values_msg)
{
  // Update the desired RC override values
  current_pwm_values_ = desired_pwm_values_msg;
}

void Bridge::enableOverride(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,  // NOLINT
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)      // NOLINT
{
  // Enable/disable publishing the override messages
  bridge_running_ = request->data;

  // Set the response according to whether or not the update was done properly
  response->success = (bridge_running_ == request->data);
}

}  // namespace blue_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<blue_bridge::Bridge>());
  rclcpp::shutdown();
  return 0;
}
