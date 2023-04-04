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

namespace blue::bridge
{

Bridge::Bridge()
: Node("blue_bridge"),
  bridge_running_(false)
{
  double override_freq = 250;
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.name = "override_freq";
    desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    desc.description =
      "The rate that the PWM inputs should be published at. This must be at least 50Hz to "
      "override the RC inputs. If the control loop frequency is greater than 50Hz, this should "
      "match the rate of the control loop.";
    desc.read_only = true;
    override_freq = declare_parameter(desc.name, override_freq, desc);
  }

  if (override_freq < 50) {
    throw std::invalid_argument(
      "The override frequency must be greater than 50Hz to override the RC inputs!");
  }

  // Set the PWM values that should be used when overriding the RC inputs
  rc_override_sub_ = this->create_subscription<mavros_msgs::msg::OverrideRCIn>(
    "/blue/rc/override", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
    [this](mavros_msgs::msg::OverrideRCIn::ConstSharedPtr pwm) -> void {  // NOLINT
      current_pwm_values_ = *pwm;
    });

  rc_override_pub_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>(
    "/mavros/rc/override", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

  // Publish the Override RC values at the specified frequency
  timer_ = create_wall_timer(std::chrono::duration<double>(1 / override_freq), [this]() -> void {
    // Only publish the override values if the bridge has been enabled
    if (active() && (rc_override_pub_->get_subscription_count() > 0)) {
      rc_override_pub_->publish(current_pwm_values_);
    }
  });

  // Enable/disable RC override
  enable_override_service_ = this->create_service<std_srvs::srv::SetBool>(
    "/blue/rc/override/enable",
    [this](
      const std::shared_ptr<std_srvs::srv::SetBool::Request> & request,
      const std::shared_ptr<std_srvs::srv::SetBool::Response> & response) -> void {
      bridge_running_ = request->data;
      response->success = (bridge_running_ == request->data);
    });
}

bool Bridge::active() const { return bridge_running_; }

}  // namespace blue::bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<blue::bridge::Bridge>());
  rclcpp::shutdown();
  return 0;
}
