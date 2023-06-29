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

#include "blue_dynamics/hydrodynamics.hpp"

namespace blue::dynamics
{

[[nodiscard]] Eigen::Matrix3d createSkewSymmetricMatrix(double a1, double a2, double a3)
{
  Eigen::Matrix3d mat;
  mat << 0, -a3, a2, a3, 0, -a1, -a2, a1, 0;

  return mat;
}

[[nodiscard]] Eigen::Matrix3d createSkewSymmetricMatrix(const Eigen::Vector3d & vec)
{
  Eigen::Matrix3d mat;
  mat << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;

  return mat;
}

Inertia::Inertia(
  double mass, const Eigen::Vector3d & inertia_tensor_coeff,
  const Eigen::Vector6d & added_mass_coeff)
{
  Eigen::Matrix6d rigid_body = Eigen::Matrix6d::Zero();

  rigid_body.topLeftCorner(3, 3) = mass * Eigen::Matrix3d::Identity();
  rigid_body.bottomRightCorner(3, 3) = inertia_tensor_coeff.asDiagonal().toDenseMatrix();

  Eigen::Matrix6d added_mass = -added_mass_coeff.asDiagonal().toDenseMatrix();

  inertia_matrix_ = rigid_body + added_mass;
}

[[nodiscard]] Eigen::Matrix6d Inertia::getInertia() const { return inertia_matrix_; }

Coriolis::Coriolis(
  double mass, const Eigen::Vector3d & inertia_tensor_coeff,
  const Eigen::Vector6d & added_mass_coeff)
: mass_(mass),
  moments_(inertia_tensor_coeff.asDiagonal().toDenseMatrix()),
  added_mass_coeff_(std::move(added_mass_coeff))
{
}

[[nodiscard]] Eigen::Matrix6d Coriolis::calculateCoriolis(const Eigen::Vector6d & velocity) const
{
  // The rigid body Coriolis matrix only needs the angular velocity
  const Eigen::Vector3d angular_velocity = velocity.bottomRows(3);

  return calculateRigidBodyCoriolis(angular_velocity) + calculateAddedCoriolis(velocity);
}

[[nodiscard]] Eigen::Matrix6d Coriolis::calculateRigidBodyCoriolis(
  const Eigen::Vector3d & angular_velocity) const
{
  Eigen::Matrix6d rigid = Eigen::Matrix6d::Zero();

  const Eigen::Vector3d moments_v2 = moments_ * angular_velocity;

  rigid.topLeftCorner(3, 3) = mass_ * createSkewSymmetricMatrix(angular_velocity);
  rigid.bottomRightCorner(3, 3) = -createSkewSymmetricMatrix(moments_v2);

  return rigid;
}

[[nodiscard]] Eigen::Matrix6d Coriolis::calculateAddedCoriolis(
  const Eigen::Vector6d & velocity) const
{
  Eigen::Matrix6d added = Eigen::Matrix6d::Zero();

  Eigen::Matrix3d linear_vel = createSkewSymmetricMatrix(
    added_mass_coeff_(0) * velocity(0), added_mass_coeff_(1) * velocity(1),
    added_mass_coeff_(2) * velocity(2));

  Eigen::Matrix3d angular_vel = createSkewSymmetricMatrix(
    added_mass_coeff_(3) * velocity(3), added_mass_coeff_(4) * velocity(4),
    added_mass_coeff_(5) * velocity(5));

  added.topRightCorner(3, 3) = linear_vel;
  added.bottomLeftCorner(3, 3) = linear_vel;
  added.bottomRightCorner(3, 3) = angular_vel;

  return added;
}

Damping::Damping(
  const Eigen::Vector6d & linear_damping_coeff, const Eigen::Vector6d & quadratic_damping_coeff)
: linear_damping_(-linear_damping_coeff.asDiagonal().toDenseMatrix()),
  quadratic_damping_coeff_(std::move(quadratic_damping_coeff))
{
}

[[nodiscard]] Eigen::Matrix6d Damping::calculateDamping(const Eigen::Vector6d & velocity) const
{
  return linear_damping_ + calculateNonlinearDamping(velocity);
}

[[nodiscard]] Eigen::Matrix6d Damping::calculateNonlinearDamping(
  const Eigen::Vector6d & velocity) const
{
  return -(quadratic_damping_coeff_.asDiagonal().toDenseMatrix() * velocity.cwiseAbs())
            .asDiagonal()
            .toDenseMatrix();
}

RestoringForces::RestoringForces(
  double weight, double buoyancy, const Eigen::Vector3d & center_of_buoyancy,
  const Eigen::Vector3d & center_of_gravity)
: weight_(weight),
  buoyancy_(buoyancy),
  center_of_buoyancy_(center_of_buoyancy),
  center_of_gravity_(center_of_gravity)
{
}

[[nodiscard]] Eigen::Vector6d RestoringForces::calculateRestoringForces(
  const Eigen::Matrix3d & rotation) const
{
  const Eigen::Vector3d fg(0, 0, weight_);
  const Eigen::Vector3d fb(0, 0, -buoyancy_);

  Eigen::Vector6d g_rb;

  g_rb.topRows(3) = rotation * (fg + fb);
  g_rb.bottomRows(3) =
    center_of_gravity_.cross(rotation * fg) + center_of_buoyancy_.cross(rotation * fb);

  g_rb *= -1;

  return g_rb;
}

CurrentEffects::CurrentEffects(const Eigen::Vector6d & current_velocity)
: current_velocity_(std::move(current_velocity))
{
}

[[nodiscard]] Eigen::Vector6d CurrentEffects::calculateCurrentEffects(
  const Eigen::Matrix3d & rotation) const
{
  Eigen::Vector6d rotated_current_effects;
  rotated_current_effects.topRows(3) = rotation * current_velocity_.topRows(3);
  rotated_current_effects.bottomRows(3) = rotation * current_velocity_.bottomRows(3);

  return rotated_current_effects;
}

HydrodynamicParameters::HydrodynamicParameters(
  Inertia inertia, Coriolis coriolis, Damping damping, RestoringForces restoring_forces,
  CurrentEffects current_effects)
: inertia(std::move(inertia)),
  coriolis(std::move(coriolis)),
  damping(std::move(damping)),
  restoring_forces(std::move(restoring_forces)),
  current_effects(std::move(current_effects))
{
}

}  // namespace blue::dynamics
