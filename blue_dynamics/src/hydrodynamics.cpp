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

#include "blue_dynamics/hydrodynamics.hpp"

namespace blue::dynamics
{

[[nodiscard]] Eigen::Matrix3d createSkewSymmetricMatrix(double a1, double a2, double a3)
{
  Eigen::Matrix3d mat;
  mat << 0, -a3, a2, a3, 0, -a1, -a2, a1, 0;

  return mat;
}

Inertia::Inertia(
  double mass, const Eigen::Vector3d & inertia_tensor_coeff,
  const Eigen::VectorXd & added_mass_coeff)
{
  // Create the rigid body mass matrix from the coefficients
  Eigen::MatrixXd rigid_body = Eigen::MatrixXd::Zero(6, 6);

  rigid_body.topLeftCorner(3, 3) = mass * Eigen::MatrixXd::Identity(3, 3);
  rigid_body.bottomRightCorner(3, 3) = inertia_tensor_coeff.asDiagonal().toDenseMatrix();

  // Create the added mass matrix from the coefficients
  Eigen::MatrixXd added_mass = -added_mass_coeff.asDiagonal().toDenseMatrix();

  // The inertia matrix `M` is the sum of the rigid body and added mass matrices (i.e., `M = M_RB +
  // M_A`)
  inertia_matrix_ = rigid_body + added_mass;
}

[[nodiscard]] Eigen::MatrixXd Inertia::getInertia() const { return inertia_matrix_; }

}  // namespace blue::dynamics
