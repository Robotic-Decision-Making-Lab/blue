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
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace blue::dynamics::test
{

TEST(VehicleDynamicsTest, CreatesSkewSymmetricMatrix)
{
  const double a1 = 1.0;
  const double a2 = 2.0;
  const double a3 = 3.0;

  Eigen::Matrix3d expected;
  expected << 0, -a3, a2, a3, 0, -a1, -a2, a1, 0;

  const Eigen::Matrix3d actual =
    blue::dynamics::VehicleDynamics::createSkewSymmetricMatrix(a1, a2, a3);

  ASSERT_TRUE(expected.isApprox(actual));
}

TEST(VehicleDynamicsTest, CalculatesRigidBodyMassMatrix)
{
  const auto moments = blue::dynamics::MomentsOfInertia(1.0, 2.0, 3.0);
  const double mass = 5.0;

  Eigen::VectorXd diagonal(6);  // NOLINT
  diagonal << mass, mass, mass, moments.x, moments.y, moments.z;

  const Eigen::MatrixXd expected = diagonal.asDiagonal().toDenseMatrix();

  const Eigen::MatrixXd actual =
    blue::dynamics::VehicleDynamics::calculateRigidBodyMassMatrix(mass, moments);

  ASSERT_TRUE(expected.isApprox(actual));
}

TEST(VehicleDynamicsTest, CalculatesAddedMassMatrix)
{
  const blue::dynamics::AddedMass added_mass =
    blue::dynamics::AddedMass(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);

  const Eigen::MatrixXd expected = -added_mass.toMatrix();

  const Eigen::MatrixXd actual =
    blue::dynamics::VehicleDynamics::calculateAddedMassMatrix(added_mass);

  ASSERT_TRUE(expected.isApprox(actual));
}

TEST(VehicleDynamicsTest, CalculatesRigidBodyCoriolisMatrix)
{
  auto moments = blue::dynamics::MomentsOfInertia(1.0, 2.0, 3.0);
  const double mass = 5.0;

  geometry_msgs::msg::TwistStamped velocity;
  velocity.twist.angular.x = 1.0;
  velocity.twist.angular.y = 2.0;
  velocity.twist.angular.z = 3.0;

  Eigen::MatrixXd expected(6, 6);
  expected.topRightCorner(3, 3) = Eigen::MatrixXd::Zero(3, 3);
  expected.bottomLeftCorner(3, 3) = Eigen::MatrixXd::Zero(3, 3);
  expected.topLeftCorner(3, 3) =
    mass * blue::dynamics::VehicleDynamics::createSkewSymmetricMatrix(
             velocity.twist.angular.x, velocity.twist.angular.y, velocity.twist.angular.z);

  Eigen::Vector3d v2(3);
  v2 << velocity.twist.angular.x, velocity.twist.angular.y, velocity.twist.angular.z;

  const Eigen::Vector3d moments_v2 = moments.toMatrix() * v2;

  expected.bottomRightCorner(3, 3) = -blue::dynamics::VehicleDynamics::createSkewSymmetricMatrix(
    moments_v2(0), moments_v2(1), moments_v2(2));

  Eigen::MatrixXd actual =
    blue::dynamics::VehicleDynamics::calculateRigidBodyCoriolisMatrix(mass, moments, velocity);

  ASSERT_TRUE(expected.isApprox(actual));
}

TEST(VehicleDynamicsTest, CalculatesAddedCoriolisMatrix)
{
  const blue::dynamics::AddedMass added_mass =
    blue::dynamics::AddedMass(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);

  geometry_msgs::msg::TwistStamped velocity;
  velocity.twist.linear.x = 1.0;
  velocity.twist.linear.y = 2.0;
  velocity.twist.linear.z = 3.0;
  velocity.twist.angular.x = 1.0;
  velocity.twist.angular.y = 2.0;
  velocity.twist.angular.z = 3.0;

  Eigen::MatrixXd linear = blue::dynamics::VehicleDynamics::createSkewSymmetricMatrix(
    added_mass.x * velocity.twist.linear.x, added_mass.y * velocity.twist.linear.y,
    added_mass.z * velocity.twist.linear.z);

  Eigen::MatrixXd expected(6, 6);
  expected.topLeftCorner(3, 3) = Eigen::MatrixXd::Zero(3, 3);
  expected.topRightCorner(3, 3) = linear;
  expected.bottomLeftCorner(3, 3) = linear;
  expected.bottomRightCorner(3, 3) = blue::dynamics::VehicleDynamics::createSkewSymmetricMatrix(
    added_mass.k * velocity.twist.angular.x, added_mass.m * velocity.twist.angular.y,
    added_mass.n * velocity.twist.angular.z);

  Eigen::MatrixXd actual =
    blue::dynamics::VehicleDynamics::calculateAddedCoriolixMatrix(added_mass, velocity);

  ASSERT_TRUE(expected.isApprox(actual));
}

TEST(VehicleDynamicsTest, CalculatesLinearDampingMatrix)
{
  const auto linear = blue::dynamics::LinearDamping(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
  const Eigen::MatrixXd expected = -linear.toMatrix();

  const Eigen::MatrixXd actual =
    blue::dynamics::VehicleDynamics::calculateLinearDampingMatrix(linear);

  ASSERT_TRUE(expected.isApprox(actual));
}

TEST(VehicleDynamicsTest, CalculatesNonlinearDampingMatrix)
{
  geometry_msgs::msg::TwistStamped velocity;
  velocity.twist.linear.x = -1.0;
  velocity.twist.linear.y = -2.0;
  velocity.twist.linear.z = -3.0;
  velocity.twist.angular.x = -1.0;
  velocity.twist.angular.y = -2.0;
  velocity.twist.angular.z = -3.0;

  Eigen::VectorXd velocity_vect(6);  // NOLINT
  velocity_vect << std::abs(velocity.twist.linear.x), std::abs(velocity.twist.linear.y),
    std::abs(velocity.twist.linear.z), std::abs(velocity.twist.angular.x),
    std::abs(velocity.twist.angular.y), std::abs(velocity.twist.angular.z);

  const auto nonlinear = blue::dynamics::NonlinearDamping(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
  const Eigen::MatrixXd expected =
    -(nonlinear.toMatrix() * velocity_vect).asDiagonal().toDenseMatrix();

  const Eigen::MatrixXd actual =
    blue::dynamics::VehicleDynamics::calculateNonlinearDampingMatrix(nonlinear, velocity);

  ASSERT_TRUE(expected.isApprox(actual));
}

TEST(VehicleDynamicsTest, CalculatesRestoringForcesVector)
{
  const double weight = 10.0;
  const double buoyancy = 7.0;
  auto pose = geometry_msgs::msg::PoseStamped();

  const auto center_of_gravity = blue::dynamics::CenterOfGravity(0.1, 0.1, 0.3);

  // Buoyancy is coincident with the center of gravity
  const auto center_of_buoyancy = blue::dynamics::CenterOfBuoyancy(0.1, 0.1, 0.3);

  // No rotation
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;

  Eigen::VectorXd expected(6);  // NOLINT

  Eigen::Vector3d fg(0, 0, weight);     // NOLINT
  Eigen::Vector3d fb(0, 0, -buoyancy);  // NOLINT

  expected.topRows(3) = fg + fb;
  expected.bottomRows(3) = static_cast<Eigen::Vector3d>(center_of_gravity.toVector()).cross(fg) +
                           static_cast<Eigen::Vector3d>(center_of_buoyancy.toVector()).cross(fb);
  expected *= -1;

  const auto vehicle_dynamics = blue::dynamics::VehicleDynamics(
    0.0, weight, buoyancy, blue::dynamics::MomentsOfInertia(), blue::dynamics::AddedMass(),
    blue::dynamics::LinearDamping(), blue::dynamics::NonlinearDamping(), center_of_buoyancy,
    center_of_gravity);

  Eigen::VectorXd actual = vehicle_dynamics.calculateRestoringForcesVector(pose);

  ASSERT_TRUE(expected.isApprox(actual));
}

}  // namespace blue::dynamics::test

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();

  return result;
}
