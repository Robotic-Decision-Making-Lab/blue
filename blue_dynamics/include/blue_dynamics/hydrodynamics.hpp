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

namespace blue::dynamics
{

[[nodiscard]] static Eigen::Matrix3d createSkewSymmetricMatrix(double a1, double a2, double a3);
[[nodiscard]] static Eigen::Matrix3d createSkewSymmetricMatrix(const Eigen::Vector3d & vec);

class Inertia
{
public:
  Inertia(
    double mass, const Eigen::Vector3d & inertia_tensor_coeff,
    const Eigen::VectorXd & added_mass_coeff);

  [[nodiscard]] Eigen::MatrixXd getInertia() const;

private:
  Eigen::MatrixXd inertia_matrix_;
};

class Coriolis
{
public:
  Coriolis(
    double mass, const Eigen::Vector3d & inertia_tensor_coeff,
    const Eigen::VectorXd & added_mass_coeff);

  [[nodiscard]] Eigen::MatrixXd calculateCoriolis(const Eigen::VectorXd & velocity) const;
  [[nodiscard]] Eigen::MatrixXd calculateCoriolisDot(const Eigen::VectorXd & accel) const;

private:
  double mass_;
  Eigen::Matrix3d moments_;
  Eigen::VectorXd added_mass_coeff_;

  [[nodiscard]] Eigen::MatrixXd calculateRigidBodyCoriolis(
    const Eigen::Vector3d & angular_velocity) const;
  [[nodiscard]] Eigen::MatrixXd calculateAddedCoriolis(const Eigen::VectorXd & velocity) const;
};

class Damping
{
public:
  Damping(
    const Eigen::VectorXd & linear_damping_coeff, const Eigen::VectorXd & quadratic_damping_coeff);

  [[nodiscard]] Eigen::MatrixXd calculateDamping(const Eigen::VectorXd & velocity) const;
  [[nodiscard]] Eigen::MatrixXd calculateDampingDot(const Eigen::VectorXd & accel) const;

private:
  Eigen::MatrixXd linear_damping_;
  Eigen::VectorXd quadratic_damping_coeff_;

  [[nodiscard]] Eigen::MatrixXd calculateNonlinearDamping(const Eigen::VectorXd & velocity) const;
};

class RestoringForces
{
public:
  RestoringForces(
    double weight, double buoyancy, const Eigen::Vector3d & center_of_buoyancy,
    const Eigen::Vector3d & center_of_gravity);

  [[nodiscard]] Eigen::VectorXd calculateRestoringForces(const Eigen::Matrix3d & rotation) const;
  [[nodiscard]] Eigen::VectorXd calculateRestoringForcesDot(
    const Eigen::Matrix3d & rotation, const Eigen::Vector3d & angular_velocity) const;

private:
  double weight_;
  double buoyancy_;
  Eigen::Vector3d center_of_buoyancy_;
  Eigen::Vector3d center_of_gravity_;
};

class CurrentEffects
{
public:
  explicit CurrentEffects(const Eigen::VectorXd & current_velocity);

  [[nodiscard]] Eigen::VectorXd calculateCurrentEffects(const Eigen::Matrix3d & rotation) const;

private:
  Eigen::VectorXd current_velocity_;
};

struct HydrodynamicParameters
{
  Inertia inertia;
  Coriolis coriolis;
  Damping damping;
  RestoringForces restoring_forces;
  CurrentEffects current_effects;

  HydrodynamicParameters(
    const Inertia & inertia, const Coriolis & coriolis, const Damping & damping,
    const RestoringForces & restoring_forces, const CurrentEffects & current_effects);
};

}  // namespace blue::dynamics
