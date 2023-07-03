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
#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "blue_dynamics/hydrodynamics.hpp"
#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "mavros_msgs/msg/override_rc_in.hpp"
#include "mavros_msgs/srv/message_interval.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace blue::control
{

/**
 * @brief A base class for custom BlueROV2 controllers.
 */
class Controller : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Controller object.
   *
   * @param node_name The name of the ROS node.
   */
  explicit Controller(std::string node_name);

protected:
  /**
   * @brief Function executed when the controller is armed.
   */
  virtual void onArm() = 0;

  /**
   * @brief Function executed when the controller is disarmed.
   */
  virtual void onDisarm() = 0;

  /**
   * @brief Update the current control inputs to the thrusters
   *
   * @return mavros_msgs::msg::OverrideRCIn
   */
  virtual mavros_msgs::msg::OverrideRCIn calculateControlInput() = 0;

  /**
   * @brief A collection of the hydrodynamic parameters for the BlueROV2.
   */
  blue::dynamics::HydrodynamicParameters hydrodynamics_;

  /**
   * @brief The thruster configuration matrix for the BlueROV2.
   */
  Eigen::MatrixXd tcm_;

  /**
   * @brief The current state of the battery.
   *
   * @note This can be used to approximate the thrust curve according to the current battery
   * voltage.
   */
  sensor_msgs::msg::BatteryState battery_state_;

  /**
   * @brief The current pose and twist of the BlueROV2.
   */
  nav_msgs::msg::Odometry odom_;

  /**
   * @brief The current acceleration of the BlueROV2.
   *
   * @note This is not provided by ArduSub directly and is calculated using finite differencing.
   */
  geometry_msgs::msg::Accel accel_;

  /**
   * @brief The total time (s) between control loop iterations
   *
   * @note This can be useful when calculating integral terms for the controller.
   */
  double dt_{0.0};

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

private:
  /**
   * @brief Enable/disable the controller.
   *
   * @details This method enables/disables sending RC Override messages to the BlueROV2.
   *
   * @param request The request to enable/disable the controller.
   * @param response The result of the arming request.
   */
  void armControllerCb(
    std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  /**
   * @brief Handle the incoming odometry messages.
   *
   * @note This message will be published after MAVROS and all of it's plugins are loaded.
   *
   * @param msg The current odometry message.
   */
  void updateOdomCb(nav_msgs::msg::Odometry::ConstSharedPtr msg);

  /**
   * @brief Set custom MAVLink message rates.
   *
   * @note This is inspired by the Orca4 project:
   * https://github.com/clydemcqueen/orca4/tree/main
   *
   * @param msg_ids The message IDs to set the rates for.
   * @param rates The frequencies that the FCU should send the messages at.
   */
  void setMessageRates(const std::vector<int64_t> & msg_ids, const std::vector<float> & rates);

  /**
   * @brief Set the rate of a MAVLink message.
   *
   * @param msg_id The message ID to set the rate for.
   * @param rate The frequency that the FCU should send the message at.
   */
  void setMessageRate(int64_t msg_id, float rate);

  // Manages whether or not control inputs are sent to ArduSub
  bool armed_{false};

  // TF2
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // Publishers
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_override_pub_;
  rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr accel_pub_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ardu_pose_sub_;

  // Callback groups
  rclcpp::CallbackGroup::SharedPtr control_loop_cb_group_;

  // Timers
  rclcpp::TimerBase::SharedPtr control_loop_timer_;
  rclcpp::TimerBase::SharedPtr set_message_rate_timer_;

  // Services
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arm_srv_;

  // Service clients
  rclcpp::Client<mavros_msgs::srv::MessageInterval>::SharedPtr set_msg_interval_client_;
};

}  // namespace blue::control
