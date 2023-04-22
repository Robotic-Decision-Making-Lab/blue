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

#include "blue_dynamics/vehicle_dynamics.hpp"

#include <cmath>

namespace blue::dynamics
{

VehicleDynamics::VehicleDynamics(
  double mass, double weight, double buoyancy, const MomentsOfInertia & moments,
  const AddedMass & added_mass, const LinearDamping & linear_damping,
  const NonlinearDamping & quadratic_damping, const CenterOfBuoyancy & center_of_buoyancy)
: mass(mass),
  weight(weight),
  buoyancy(buoyancy),
  moments(std::move(moments)),
  added_mass(std::move(added_mass)),
  linear_damping(std::move(linear_damping)),
  quadratic_damping(std::move(quadratic_damping)),
  center_of_buoyancy(std::move(center_of_buoyancy))
{
}

[[nodiscard]] Eigen::Matrix3d VehicleDynamics::createSkewSymmetricMatrix(
  double a1, double a2, double a3)
{
  Eigen::Matrix3d mat;
  mat << 0, -a3, a2, a3, 0, -a1, -a2, a1, 0;

  return mat;
}

[[nodiscard]] Eigen::MatrixXd VehicleDynamics::calculateInertiaMatrix() const
{
  return calculateRigidBodyMatrix(mass, moments) + calculateAddedMassMatrix(added_mass);
}

[[nodiscard]] Eigen::MatrixXd VehicleDynamics::calculateRigidBodyMatrix(
  double mass, const MomentsOfInertia & moments)
{
  Eigen::MatrixXd mat(6, 6);

  mat.topLeftCorner(3, 3) = mass * Eigen::MatrixXd::Identity(3, 3);
  mat.bottomRightCorner(3, 3) = moments.toMatrix();

  return mat;
}

[[nodiscard]] Eigen::MatrixXd VehicleDynamics::calculateAddedMassMatrix(
  const AddedMass & added_mass)
{
  return added_mass.toMatrix();
}

[[nodiscard]] Eigen::MatrixXd VehicleDynamics::calculateCoriolisMatrix(
  const geometry_msgs::msg::TwistStamped & velocity) const
{
  return calculateRigidBodyCoriolisMatrix(mass, moments, velocity) +
         calculateHydrodynamicCoriolixMatrix(added_mass, velocity);
}

[[nodiscard]] Eigen::MatrixXd VehicleDynamics::calculateRigidBodyCoriolisMatrix(
  double mass, const MomentsOfInertia & moments, const geometry_msgs::msg::TwistStamped & velocity)
{
  Eigen::MatrixXd mat(6, 6);

  Eigen::Matrix3d linear_vel =
    mass * createSkewSymmetricMatrix(
             velocity.twist.linear.x, velocity.twist.linear.y, velocity.twist.linear.z);

  Eigen::Matrix3d angular_vel = createSkewSymmetricMatrix(
    moments.x * velocity.twist.angular.x, moments.y * velocity.twist.angular.y,
    moments.z * velocity.twist.angular.z);

  mat.topRightCorner(3, 3) = linear_vel;
  mat.bottomLeftCorner(3, 3) = linear_vel;
  mat.bottomRightCorner(3, 3) = angular_vel;

  return mat;
}

[[nodiscard]] Eigen::MatrixXd VehicleDynamics::calculateHydrodynamicCoriolixMatrix(
  const AddedMass & added_mass, const geometry_msgs::msg::TwistStamped & velocity)
{
  Eigen::MatrixXd mat(6, 6);

  Eigen::Matrix3d linear_vel = createSkewSymmetricMatrix(
    added_mass.x * velocity.twist.linear.x, added_mass.y * velocity.twist.linear.y,
    added_mass.z * velocity.twist.linear.z);

  Eigen::Matrix3d angular_vel = createSkewSymmetricMatrix(
    added_mass.k * velocity.twist.angular.x, added_mass.m * velocity.twist.angular.y,
    added_mass.n * velocity.twist.angular.z);

  mat.topRightCorner(3, 3) = linear_vel;
  mat.bottomLeftCorner(3, 3) = linear_vel;
  mat.bottomRightCorner(3, 3) = angular_vel;

  return mat;
}

[[nodiscard]] Eigen::MatrixXd VehicleDynamics::calculateDampingMatrix(
  const geometry_msgs::msg::TwistStamped & velocity) const
{
  return calculateLinearDampingMatrix(linear_damping) +
         calculateNonlinearDampingMatrix(quadratic_damping, velocity);
}

[[nodiscard]] Eigen::MatrixXd VehicleDynamics::calculateLinearDampingMatrix(
  const LinearDamping & linear_damping)
{
  return -1 * linear_damping.toMatrix();
}

[[nodiscard]] Eigen::MatrixXd VehicleDynamics::calculateNonlinearDampingMatrix(
  const NonlinearDamping & quadratic_damping, const geometry_msgs::msg::TwistStamped & velocity)
{
  Eigen::VectorXd vec(6);
  vec << std::abs(velocity.twist.linear.x), std::abs(velocity.twist.linear.y),
    std::abs(velocity.twist.linear.z), std::abs(velocity.twist.angular.x),
    std::abs(velocity.twist.angular.y), std::abs(velocity.twist.angular.z);

  return -1 * quadratic_damping.toMatrix() * vec;
}

[[nodiscard]] Eigen::VectorXd VehicleDynamics::calculateRestoringForcesVector(
  const geometry_msgs::msg::PoseStamped & pose) const
{
}
}  // namespace blue::dynamics
