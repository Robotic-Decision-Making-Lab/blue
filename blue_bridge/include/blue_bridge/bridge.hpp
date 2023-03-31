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

#include "mavros_msgs/msg/override_rc_in.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace blue::bridge
{

class Bridge : public rclcpp::Node
{
public:
  Bridge();

  [[nodiscard]] bool active() const;

private:
  void publishOverrideRcInCb() const;
  void setCurrentPwmValuesCb(mavros_msgs::msg::OverrideRCIn::ConstSharedPtr pwm);
  void enableOverrideCb(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> & request,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> & response);

  bool bridge_running_;
  mavros_msgs::msg::OverrideRCIn current_pwm_values_;

  rclcpp::Subscription<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_override_sub_;
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_override_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_override_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace blue::bridge
