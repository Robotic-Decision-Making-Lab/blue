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

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>

#include "blue_dynamics/hydrodynamics.hpp"

namespace blue::dynamics::test
{

using blue::dynamics::Coriolis;
using blue::dynamics::Damping;
using blue::dynamics::Inertia;
using blue::dynamics::Matrix6d;
using blue::dynamics::RestoringForces;
using blue::dynamics::Vector6d;

TEST(HydrodynamicsTest, TestInertia)
{
  const double mass = 5.0;

  const Eigen::Vector3d inertia_coeff(1, 2, 3);

  Vector6d added_mass_coeff;  // NOLINT
  added_mass_coeff << 1, 2, 3, 4, 5, 6;

  const Inertia inertia = Inertia(mass, inertia_coeff, added_mass_coeff);

  Vector6d expected;  // NOLINT
  expected << 4, 3, 2, -3, -3, -3;

  const Matrix6d expected_matrix = expected.asDiagonal().toDenseMatrix();
  const Matrix6d actual = inertia.getInertia();

  ASSERT_TRUE(actual.isApprox(expected_matrix));
}

TEST(HydrodynamicsTest, TestCoriolis)
{
  const double mass = 5.0;

  const Eigen::Vector3d inertia_coeff(1, 2, 3);

  Vector6d added_mass_coeff;  // NOLINT
  added_mass_coeff << 1, 2, 3, 4, 5, 6;

  const Coriolis coriolis = Coriolis(mass, inertia_coeff, added_mass_coeff);

  Vector6d velocity;  // NOLINT
  velocity << 2, 2, 2, 2, 2, 2;

  Eigen::Matrix3d tl_rb;
  tl_rb << 0, -10, 10, 10, 0, -10, -10, 10, 0;

  Eigen::Matrix3d br_rb;
  br_rb << 0, 6, -4, -6, 0, 2, 4, -2, 0;

  Matrix6d rigid = Matrix6d::Zero();  // NOLINT
  rigid.topLeftCorner(3, 3) = tl_rb;
  rigid.bottomRightCorner(3, 3) = br_rb;

  Eigen::Matrix3d tr_bl_a;
  tr_bl_a << 0, -6, 4, 6, 0, -2, -4, 2, 0;

  Eigen::Matrix3d br_a;
  br_a << 0, -12, 10, 12, 0, -8, -10, 8, 0;

  Matrix6d added = Matrix6d::Zero();  // NOLINT
  added.topRightCorner(3, 3) = tr_bl_a;
  added.bottomLeftCorner(3, 3) = tr_bl_a;
  added.bottomRightCorner(3, 3) = br_a;

  Matrix6d expected = rigid + added;

  const Matrix6d actual = coriolis.calculateCoriolis(velocity);

  ASSERT_TRUE(actual.isApprox(expected));
}

TEST(HydrodynamicsTest, TestDamping)
{
  Vector6d linear_damping_coeff;  // NOLINT
  linear_damping_coeff << 1, 2, 3, 4, 5, 6;

  Vector6d quadratic_damping_coeff;  // NOLINT
  quadratic_damping_coeff << 1, 2, 3, 4, 5, 6;

  const Damping damping = Damping(linear_damping_coeff, quadratic_damping_coeff);

  Vector6d velocity;  // NOLINT
  velocity << 2, 2, 2, 2, 2, 2;

  Vector6d linear;  // NOLINT
  linear << -1, -2, -3, -4, -5, -6;

  Vector6d quadratic;  // NOLINT
  quadratic << -2, -4, -6, -8, -10, -12;

  const Matrix6d expected =
    linear.asDiagonal().toDenseMatrix() + quadratic.asDiagonal().toDenseMatrix();
  const Matrix6d actual = damping.calculateDamping(velocity);

  ASSERT_TRUE(actual.isApprox(expected));
}

TEST(HydrodynamicsTest, TestRestoringForces)
{
  const double weight = 5.0;
  const double buoyancy = 7.0;

  const Eigen::Vector3d center_of_buoyancy(1, 2, 3);
  const Eigen::Vector3d center_of_gravity(1, 2, 3);

  const RestoringForces restoring_forces =
    RestoringForces(weight, buoyancy, center_of_buoyancy, center_of_gravity);

  const Eigen::Quaterniond orientation(1, 0, 0, 0);
  const Eigen::Matrix3d rot = orientation.toRotationMatrix();

  Vector6d expected;  // NOLINT
  expected << 0, 0, 2, 4, -2, 0;

  const Vector6d actual = restoring_forces.calculateRestoringForces(rot);

  ASSERT_TRUE(actual.isApprox(expected));
}

}  // namespace blue::dynamics::test

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();

  return result;
}
