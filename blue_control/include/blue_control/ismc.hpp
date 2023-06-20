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
#include "blue_msgs/msg/reference.hpp"
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
  mavros_msgs::msg::OverrideRCIn update() override;

private:
  // Hyperparameters used by the ISMC
  blue::dynamics::Vector6d total_velocity_error_;
  blue::dynamics::Matrix6d convergence_rate_;
  double sliding_gain_;
  double boundary_thickness_;

  bool use_battery_state_;
  blue_msgs::msg::Reference cmd_;
  rclcpp::Subscription<blue_msgs::msg::Reference>::SharedPtr cmd_sub_;
};

}  // namespace blue::control
