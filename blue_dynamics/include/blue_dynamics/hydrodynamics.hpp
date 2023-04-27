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

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace blue::dynamics
{

/**
 * @brief Create a skew-symmetric matrix from the provided coefficients.
 *
 * @param a1 Coefficient one.
 * @param a2 Coefficient two.
 * @param a3 Coefficient three.
 * @return The created skew-symmetric matrix.
 */
[[nodiscard]] static Eigen::Matrix3d createSkewSymmetricMatrix(double a1, double a2, double a3);

class Intertia
{
public:
  Intertia(double mass, const Eigen::Matrix3d & moments, const Eigen::MatrixXd & added_mass);

  [[nodiscard]] Eigen::MatrixXd calculateIntertia() const;

private:
  Eigen::MatrixXd inertia_matrix_;
};

class Coriolis
{
public:
  Coriolis(double mass, const Eigen::Matrix3d & moments, const Eigen::MatrixXd & added_mass);

  [[nodiscard]] Eigen::MatrixXd calculateCoriolis(const Eigen::VectorXd & velocity) const;

private:
  double mass_;
  Eigen::Matrix3d moments_;
  Eigen::MatrixXd added_mass_;

  [[nodiscard]] Eigen::MatrixXd calculateRigidBodyCoriolis(const Eigen::VectorXd & velocity) const;
  [[nodiscard]] Eigen::MatrixXd calculateAddedCoriolis(const Eigen::VectorXd & velocity) const;
};

class Damping
{
public:
  Damping(const Eigen::MatrixXd & linear_damping, const Eigen::MatrixXd & quadratic_damping);

  [[nodiscard]] Eigen::MatrixXd calculateDamping(const geometry_msgs::msg::Twist & velocity) const;

private:
  Eigen::MatrixXd linear_damping_;
  Eigen::MatrixXd quadratic_damping_;

  [[nodiscard]] Eigen::MatrixXd calculateNonlinearDamping(
    const geometry_msgs::msg::Twist & velocity) const;
};

class RestoringForces
{
public:
  RestoringForces(
    double weight, double buoyancy, const Eigen::Vector3d & center_of_buoyancy,
    const Eigen::Vector3d & center_of_gravity);

  [[nodiscard]] Eigen::VectorXd calculateRestoringForces(const Eigen::Matrix3d & rotation) const;
};

}  // namespace blue::dynamics
