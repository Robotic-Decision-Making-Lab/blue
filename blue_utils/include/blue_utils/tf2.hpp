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
#include <string>

#include "blue_utils/eigen.hpp"
#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/wrench.hpp"

/// @cond
namespace tf2
{

// Extend the tf2 namespace to include some commonly used conversions

/**
 * @brief Convert a geometry_msgs::msg::Accel message into an Eigen vector.
 *
 * @param in The Accel message to convert.
 * @param out The Eigen vector that should be populated with the Accel data.
 */
inline void fromMsg(const geometry_msgs::msg::Accel & in, Eigen::Vector6d & out)
{
  Eigen::Vector6d v;
  v << in.linear.x, in.linear.y, in.linear.z, in.angular.x, in.angular.y, in.angular.z;
  out = v;
}

/**
 * @brief Convert a geometry_msgs::msg::Wrench into an Eigen vector.
 *
 * @param in The Wrench message to convert.
 * @param out The Eigen vector that should be populated with the Wrench data.
 */
inline void fromMsg(const geometry_msgs::msg::Wrench & in, Eigen::Vector6d & out)
{
  Eigen::Vector6d v;
  v << in.force.x, in.force.y, in.force.z, in.torque.x, in.torque.y, in.torque.z;
  out = v;
}

/**
 * @brief Convert an Eigen vector into a geometry_msgs::msg::Wrench message.
 *
 * @note This method was renamed to avoid overloading conflicts with the Twist declaration. This
 * is consistent with previous naming applied in the tf2_eigen project.
 *
 * @param in The Eigen vector to convert into a Wrench message.
 * @return geometry_msgs::msg::Wrench
 */
inline geometry_msgs::msg::Wrench toMsg2(const Eigen::Vector6d & in)
{
  geometry_msgs::msg::Wrench msg;
  msg.force.x = in[0];
  msg.force.y = in[1];
  msg.force.z = in[2];
  msg.torque.x = in[3];
  msg.torque.y = in[4];
  msg.torque.z = in[5];
  return msg;
}

}  // namespace tf2
/// @endcond

namespace blue::transforms
{

// Coordinate frame IDs
const std::string kMapFrameId{"map"};
const std::string kMapNedFrameId{"map_ned"};
const std::string kBaseLinkFrameId{"base_link"};
const std::string kBaseLinkFrdFrameId{"base_link_frd"};

}  // namespace blue::transforms
