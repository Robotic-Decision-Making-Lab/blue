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

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>

#include "blue_dynamics/vehicle_dynamics.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace blue::dynamics::test
{

TEST(HydrodynamicsTest, TestInertia)
{
  const double mass = 5.0;

  Eigen::Matrix3d moments;
  moments << 1, 0, 0, 0, 2, 0, 0, 0, 3;

  Eigen::VectorXd added_mass_coefficients(6);  // NOLINT
  added_mass_coefficients << 1, 2, 3, 4, 5, 6;

  Eigen::MatrixXd added_mass = added_mass_coefficients.asDiagonal().toDenseMatrix();

  const Inertia inertia = Inertia(mass, moments, added_mass);

  const Eigen::MatrixXd inertia_matrix = inertia.getInertia();
}

TEST(HydrodynamicsTest, TestCoriolis)
{
  Eigen::Matrix3d moments;
  moments << 1, 0, 0, 0, 2, 0, 0, 0, 3;

  Eigen::VectorXd added_mass_coefficients(6);  // NOLINT
  added_mass_coefficients << 1, 2, 3, 4, 5, 6;

  Eigen::MatrixXd added_mass = added_mass_coefficients.asDiagonal().toDenseMatrix();

  const double mass = 5.0;

  const Coriolis coriolis = Coriolis(mass, moments, added_mass);

  Eigen::VectorXd velocity(6);  // NOLINT
  velocity << 1, 2, 3, 4, 5, 6;

  const Eigen::MatrixXd coriolis_matrix = coriolis.calculateCoriolis(velocity);
}

TEST(HydrodynamicsTest, TestDamping)
{
  Eigen::VectorXd linear_damping_coefficients(6);  // NOLINT
  linear_damping_coefficients << 1, 2, 3, 4, 5, 6;

  Eigen::VectorXd quadratic_damping_coefficients(6);  // NOLINT
  quadratic_damping_coefficients << 1, 2, 3, 4, 5, 6;

  const Eigen::MatrixXd linear_damping = linear_damping_coefficients.asDiagonal().toDenseMatrix();

  const Eigen::MatrixXd quadratic_damping =
    quadratic_damping_coefficients.asDiagonal().toDenseMatrix();

  const Damping damping = Damping(linear_damping, quadratic_damping);
}

TEST(HydrodynamicsTest, TestRestoringForces)
{
  const double weight = 5.0;
  const double buoyancy = 7.0;

  const Eigen::Vector3d center_of_buoyancy(1, 2, 3);
  const Eigen::Vector3d center_of_gravity(4, 5, 6);

  const RestoringForces restoring_forces =
    RestoringForces(weight, buoyancy, center_of_buoyancy, center_of_gravity);

  const Eigen::Quaterniond orientation(1, 0, 0, 0);
  const Eigen::Matrix3d rot = orientation.toRotationMatrix();

  const Eigen::VectorXd restoring_forces_vec = restoring_forces.calculateRestoringForces(rot);
}

}  // namespace blue::dynamics::test

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();

  return result;
}
