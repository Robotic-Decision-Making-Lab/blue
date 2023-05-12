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

#include "blue_control/ismc.hpp"

#include "blue_dynamics/thruster_dynamics.hpp"

namespace blue::control
{

ISMC::ISMC()
: BaseController("ismc")
{
  // Declare the ROS parameters specific to this controller
  // There are additional parameters defined by the base controller as well
  this->declare_parameter(
    "convergence_rate", std::vector<double>{100.0, 100.0, 100.0, 100.0, 100.0, 100.0});
  this->declare_parameter("sliding_gain", 0.0);
  this->declare_parameter("boundary_thickness", 0.0);

  Eigen::VectorXd convergence_diag =
    convertVectorToEigenVector(this->get_parameter("convergence_rate").as_double_array());
  convergence_rate_ = convergence_diag.asDiagonal().toDenseMatrix();
  sliding_gain_ = this->get_parameter("sliding_gain").as_double();
  boundary_thickness_ = this->get_parameter("boundary_thickness").as_double();
  total_velocity_error_ = Eigen::VectorXd::Zero(6);
}

mavros_msgs::msg::OverrideRCIn ISMC::update()
{
  Eigen::VectorXd velocity(6);  // NOLINT
  velocity << odom_->twist.twist.linear.x, odom_->twist.twist.linear.y, odom_->twist.twist.linear.z,
    odom_->twist.twist.angular.x, odom_->twist.twist.angular.y, odom_->twist.twist.angular.z;

  Eigen::Quaterniond orientation = Eigen::Quaterniond(
    odom_->pose.pose.orientation.w, odom_->pose.pose.orientation.x, odom_->pose.pose.orientation.y,
    odom_->pose.pose.orientation.z);

  // TODO(evan): Include inertia -> need desired acceleration and velocity error
  // TODO(evan): Calculate s and apply the sign function
  // TODO(evan): Include the current effects
  const Eigen::VectorXd pwms =
    hydrodynamics_.coriolis.calculateCoriolis(velocity) * velocity +
    hydrodynamics_.damping.calculateDamping(velocity) * velocity +
    hydrodynamics_.restoring_forces.calculateRestoringForces(orientation.toRotationMatrix());

  // TODO(evan): convert the pwms to an std::array
  // TODO(evan): convert to an OverrideRCIn message

  return mavros_msgs::msg::OverrideRCIn();
}

Eigen::VectorXd ISMC::calculateError(
  const Eigen::VectorXd & desired, const Eigen::VectorXd & actual)
{
  return desired - actual;
}

Eigen::VectorXd ISMC::calculateSlidingSurface(
  const Eigen::VectorXd & velocity_error, const Eigen::VectorXd & velocity_error_integral,
  const Eigen::MatrixXd & convergence_rate)
{
  return velocity_error + convergence_rate * velocity_error_integral;
}

void ISMC::applySignFunction(std::shared_ptr<Eigen::VectorXd> surface)
{ /* data */
}

}  // namespace blue::control
