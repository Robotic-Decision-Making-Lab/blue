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

#include <atomic>
#include <mutex>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/override_rc_in.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace blue::control
{

class Controller : public rclcpp::Node
{
public:
  virtual ~Controller() = default;  // NOLINT
  Controller(const std::string & node_name, const rclcpp::NodeOptions & options);

  [[nodiscard]] bool running() const;

protected:
  virtual mavros_msgs::msg::OverrideRCIn update();

  // BlueROV2 state
  geometry_msgs::msg::PoseStamped odom_pose_;
  bool running_;

private:
  void runControlLoopCb();
  void startControlCb(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> & request,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> & response);
  void setOdomPoseCb(geometry_msgs::msg::PoseStamped::ConstSharedPtr pose);

  // Subscriptions
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  // Publishers
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_override_pub_;

  // Services
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_control_;

  // PWM timer
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace blue::control
