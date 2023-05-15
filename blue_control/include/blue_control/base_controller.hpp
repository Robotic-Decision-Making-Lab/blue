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

#pragma once

#include <Eigen/Dense>
#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "blue_dynamics/hydrodynamics.hpp"
#include "blue_dynamics/thruster_dynamics.hpp"
#include "mavros_msgs/msg/override_rc_in.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace blue::control
{

/**
 * @brief Convert an `std::vector` to an `Eigen::VectorXd`.
 *
 * @note This method is useful when converting a ROS parameter that has been read as a `std::vector`
 * to an `Eigen::VectorXd`.
 *
 * @param vec The `std::vector` to convert.
 * @return An `Eigen::VectorXd` with the same values as `vec`.
 */
Eigen::VectorXd convertVectorToEigenVector(const std::vector<double> & vec);

/**
 * @brief Convert an `std::vector` to an `Eigen::MatrixXd`.
 *
 * @note This method is useful when converting a ROS parameter that has been read as a `std::vector`
 * to an `Eigen::MatrixXd`.
 *
 * @param vec The `std::vector` to convert.
 * @param rows The total number of rows in the resulting matrix.
 * @param cols The total number of columns in the resulting matrix.
 * @return An `Eigen::MatrixXd` with the same values as `vec`.
 */
Eigen::MatrixXd convertVectorToEigenMatrix(
  const std::vector<double> & vec, size_t rows, size_t cols);

/**
 * @brief A base class for custom BlueROV2 controllers.
 */
class BaseController : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new BaseController object.
   *
   * @param node_name The name of the ROS node.
   */
  explicit BaseController(const std::string & node_name);

protected:
  /**
   * @brief Update the current control inputs to the thrusters
   *
   * @return mavros_msgs::msg::OverrideRCIn
   */
  virtual mavros_msgs::msg::OverrideRCIn update() = 0;

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

  //
  /**
   * @brief The current pose and twist of the BlueROV2.
   *
   * @note It is important to note here that the pose information is provided in the inertial frame
   * and the twist is provided in the body frame. For more information on this see:
   * https://github.com/mavlink/mavros/issues/1251
   */
  nav_msgs::msg::Odometry odom_;

private:
  /**
   * @brief Enable the controller.
   *
   * @details This method enables/disables sending RC Override messages to the BlueROV2.
   *
   * @param request The request to enable/disable the controller.
   * @param response The result of the arming request.
   */
  void armControllerCb(
    std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  bool armed_;

  // Publishers
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_override_pub_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Timers
  rclcpp::TimerBase::SharedPtr control_loop_timer_;

  // Services
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arm_srv_;
};

}  // namespace blue::control
