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

#include <Eigen/Dense>

#include "blue_dynamics/hydrodynamics.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace blue::dynamics
{

struct VehicleDynamics
{
  double mass;
  double weight;
  double buoyancy;
  MomentsOfInertia moments;
  AddedMass added_mass;
  LinearDamping linear_damping;
  NonlinearDamping quadratic_damping;
  CenterOfBuoyancy center_of_buoyancy;

  VehicleDynamics(
    double mass, double weight, double buoyancy, const MomentsOfInertia & moments,
    const AddedMass & added_mass, const LinearDamping & linear_damping,
    const NonlinearDamping & quadratic_damping, const CenterOfBuoyancy & center_of_buoyancy);

  [[nodiscard]] static Eigen::Matrix3d createSkewSymmetricMatrix(double a1, double a2, double a3);

  [[nodiscard]] Eigen::MatrixXd calculateInertiaMatrix() const;

  [[nodiscard]] static Eigen::MatrixXd calculateRigidBodyMatrix(
    double mass, const MomentsOfInertia & moments);

  [[nodiscard]] static Eigen::MatrixXd calculateAddedMassMatrix(const AddedMass & added_mass);

  [[nodiscard]] Eigen::MatrixXd calculateCoriolisMatrix(
    const geometry_msgs::msg::TwistStamped & velocity) const;

  [[nodiscard]] static Eigen::MatrixXd calculateRigidBodyCoriolisMatrix(
    double mass, const MomentsOfInertia & moments,
    const geometry_msgs::msg::TwistStamped & velocity);

  [[nodiscard]] static Eigen::MatrixXd calculateHydrodynamicCoriolixMatrix(
    const AddedMass & added_mass, const geometry_msgs::msg::TwistStamped & velocity);

  [[nodiscard]] Eigen::MatrixXd calculateDampingMatrix(
    const geometry_msgs::msg::TwistStamped & velocity) const;

  [[nodiscard]] static Eigen::MatrixXd calculateLinearDampingMatrix(
    const LinearDamping & linear_damping);

  [[nodiscard]] static Eigen::MatrixXd calculateNonlinearDampingMatrix(
    const NonlinearDamping & quadratic_damping, const geometry_msgs::msg::TwistStamped & velocity);

  [[nodiscard]] Eigen::VectorXd calculateRestoringForcesVector(
    const geometry_msgs::msg::PoseStamped & pose) const;
};

}  // namespace blue::dynamics
