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

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace blue::dynamics
{

/**
 * @brief The velocity of the fluid in which the BlueROV2 is operating.
 */
struct CurrentEffects
{
  double v_cx;  // The linear flow rate in the x direction.
  double v_cy;  // The linear flow rate in the y direction.
  double v_cz;  // The linear flow rate in the z direction.
  double v_cp;  // The rotational flow rate around the x axis.
  double v_cq;  // The rotational flow rate around the y axis.
  double v_cr;  // The rotational flow rate around the z axis.

  /**
   * @brief Construct a new CurrentEffects object.
   *
   * @param v_cx The linear flow rate in the x direction.
   * @param v_cy The linear flow rate in the y direction.
   * @param v_cz The linear flow rate in the z direction.
   * @param v_cp The rotational flow rate around the x axis.
   * @param v_cq The rotational flow rate around the y axis.
   * @param v_cr The rotational flow rate around the z axis.
   */
  CurrentEffects(double v_cx, double v_cy, double v_cz, double v_cp, double v_cq, double v_cr);

  /**
   * @brief Calculate the ocean current velocity with respect to the body frame.
   *
   * @note The formula used to implement the current effect calculate is obtained from Gianluca
   * Antonelli's "Underwater Robots" in Section 2.4.3.
   *
   * @param pose The current pose of the vehicle in the inertial frame.
   * @return The ocean current velocity in the body frame.
   */
  [[nodiscard]] Eigen::VectorXd calculateCurrentEffects(
    const geometry_msgs::msg::PoseStamped & pose) const;
};

}  // namespace blue::dynamics
