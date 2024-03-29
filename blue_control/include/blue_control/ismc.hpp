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

#pragma once

#include <Eigen/Dense>

#include "blue_control/controller.hpp"
#include "blue_dynamics/thruster_dynamics.hpp"
#include "blue_utils/eigen.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "mavros_msgs/msg/override_rc_in.hpp"

namespace blue::control
{

/**
 * @brief Integral sliding mode controller for the BlueROV2.
 */
class ISMC : public Controller
{
public:
  /**
   * @brief Construct a new ISMC object.
   */
  ISMC();

protected:
  void onArm() override;
  void onDisarm() override;
  mavros_msgs::msg::OverrideRCIn calculateControlInput() override;

private:
  // ISMC gains
  Eigen::Matrix6d integral_gain_;
  Eigen::Matrix6d proportional_gain_;
  Eigen::Matrix6d derivative_gain_;
  double sliding_gain_;
  double boundary_thickness_;

  // Error terms
  Eigen::Vector6d initial_velocity_error_;
  Eigen::Vector6d initial_acceleration_error_;
  Eigen::Vector6d total_velocity_error_;

  // Control whether or not the battery state is used when converting thrust to PWM
  bool use_battery_state_;

  // Reference signal
  geometry_msgs::msg::Twist cmd_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr desired_wrench_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_error_pub_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
};

}  // namespace blue::control
