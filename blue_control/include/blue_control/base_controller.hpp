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

#include <Eigen/Dense>
#include <atomic>
#include <string>
#include <vector>

#include "blue_dynamics/hydrodynamics.hpp"
#include "blue_dynamics/thruster_dynamics.hpp"
#include "mavros_msgs/msg/override_rc_in.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

namespace blue::control
{

Eigen::VectorXd convertVectorToEigenVector(const std::vector<double> & vec);
Eigen::MatrixXd convertVectorToEigenMatrix(
  const std::vector<double> & vec, size_t rows, size_t cols);

class BaseController : public rclcpp::Node
{
public:
  explicit BaseController(const std::string & node_name);

protected:
  virtual mavros_msgs::msg::OverrideRCIn update() = 0;

  blue::dynamics::HydrodynamicParameters hydrodynamics_;
  Eigen::MatrixXd tcm_;
  sensor_msgs::msg::BatteryState battery_state_;

  // It is important to note here that the pose information is provided in the inertial frame
  // and the twist is provided in the body frame. For more information on this see:
  // https://github.com/mavlink/mavros/issues/1251
  nav_msgs::msg::Odometry odom_;

private:
  // Publishers
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_override_pub_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Timers
  rclcpp::TimerBase::SharedPtr control_loop_timer_;
};

}  // namespace blue::control
