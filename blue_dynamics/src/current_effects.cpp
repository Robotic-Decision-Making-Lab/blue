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

#include "blue_dynamics/current_effects.hpp"

namespace blue::dynamics
{

CurrentEffects::CurrentEffects(
  double v_cx, double v_cy, double v_cz, double v_cp = 0, double v_cq = 0, double v_cr = 0)
: v_cx(v_cx),
  v_cy(v_cy),
  v_cz(v_cz),
  v_cp(v_cp),
  v_cq(v_cq),
  v_cr(v_cr)
{
}

[[nodiscard]] Eigen::VectorXd CurrentEffects::calculateCurrentEffects(
  const geometry_msgs::msg::PoseStamped & pose) const
{
  Eigen::Quaterniond q(
    pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y,
    pose.pose.orientation.z);

  Eigen::Matrix3d rot = q.toRotationMatrix();

  Eigen::VectorXd vec(6);  // NOLINT
  vec << v_cx, v_cy, v_cz, v_cp, v_cq, v_cr;

  return rot * vec;
}

}  // namespace blue::dynamics
