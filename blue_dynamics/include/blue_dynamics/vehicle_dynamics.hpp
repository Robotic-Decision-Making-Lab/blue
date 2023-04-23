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

/**
 * @brief The hydrodynamic model for an underwater vehicle.
 *
 * @note This implementation draws from Thor I. Fossen's "Handbook of Marine Craft Hydrodynamics
 * and Motion Control" (2011) and Gianluca Antonelli's "Underwater Robots" (2014) for the
 * hydrodynamic model. Relevant equations and definitions used are documented with each method.
 *
 */
struct VehicleDynamics
{
  double mass;                   // The total mass of the vehicle (kg).
  double weight;                 // The weight of the vehicle in the inertial frame.
  double buoyancy;               // The buoyance force acting on the vehicle in the inertial frame.
  MomentsOfInertia moments;      // The moments of inertia.
  AddedMass added_mass;          // The added mass coefficients.
  LinearDamping linear_damping;  // The linear damping coefficients.
  NonlinearDamping quadratic_damping;   // The nonlinear damping coefficients.
  CenterOfBuoyancy center_of_buoyancy;  // The center of buoyancy.
  CenterOfGravity center_of_gravity;    // The center of gravity.

  /**
   * @brief Create a new VehicleDynamics object.
   *
   * @param mass The total mass of the vehicle (kg).
   * @param weight The weight of the vehicle in the inertial frame. This is denoted as `W` in
   * Section 2.5 of Gianluca Antonelli's "Underwater Robots".
   * @param buoyancy The buoyancy force acting on the vehicle. This is denoted as `B` in Section 2.5
   * of Gianluca Antonelli's "Underwater Robots".
   * @param moments The moments of inertia.
   * @param added_mass The added mass coefficients. These are defined in Section 2.4.1 of Gianluca
   * Antonelli's "Underwater Robots".
   * @param linear_damping The rigid body linear damping coefficients. These are defined in Section
   * 2.4.2 of Gianluca Antonelli's "Underwater Robots".
   * @param quadratic_damping The rigid body nonlinear damping coefficients. These are defined in
   * Section 2.4.2 of Gianluca Antonelli's "Underwater Robots".
   * @param center_of_buoyancy The center of buoyancy.
   * @param center_of_gravity The center of gravity relative to the body frame.
   */
  VehicleDynamics(
    double mass, double weight, double buoyancy, const MomentsOfInertia & moments,
    const AddedMass & added_mass, const LinearDamping & linear_damping,
    const NonlinearDamping & quadratic_damping, const CenterOfBuoyancy & center_of_buoyancy,
    const CenterOfGravity & center_of_gravity);

  /**
   * @brief Create a skew-symmetric matrix from the provided coefficients.
   *
   * @note This is also referred to as the "Cross-Product Operator" by some textbooks.
   *
   * @param a1 Coefficient one.
   * @param a2 Coefficient two.
   * @param a3 Coefficient three.
   * @return Eigen::Matrix3d
   */
  [[nodiscard]] static Eigen::Matrix3d createSkewSymmetricMatrix(double a1, double a2, double a3);

  /**
   * @brief Calculate the vehicle's inertia matrix.
   *
   * @note The inertia matrix `M` is the sum of the rigid body mass `M_RB` and the added mass `M_A`
   * such that `M = M_RB + M_A`.
   *
   * @return Eigen::MatrixXd
   */
  [[nodiscard]] Eigen::MatrixXd calculateInertiaMatrix() const;

  /**
   * @brief Calculate the rigid body inertia matrix.
   *
   * @note The definition used for the rigid body inertia matrix `M_RB` is provided by
   * Thor I. Fossen's textbook "Handbook of Marine Craft Hydrodynamics and Motion Control" in
   * Equation 3.44. Note that, in this model, we define the body frame to be coincident with the
   * center of mass, such that r_g^b = 0. The result is that `M_RB` is a diagonal matrix.
   *
   * @param mass The total mass of the vehicle (kg).
   * @param moments The moments of inertia. This is assumed to be a diagonal matrix.
   * @return Eigen::MatrixXd
   */
  [[nodiscard]] static Eigen::MatrixXd calculateRigidBodyMassMatrix(
    double mass, const MomentsOfInertia & moments);

  /**
   * @brief Calculate the added mass inertia matrix.
   *
   * @note The definition used for the added mass inertia matrix `M_A` is provided by Gianluca
   * Antonelli's textbook "Underwater Robots" in Section 2.4.1.
   *
   * @param added_mass The added mass coefficients.
   * @return Eigen::MatrixXd
   */
  [[nodiscard]] static Eigen::MatrixXd calculateAddedMassMatrix(const AddedMass & added_mass);

  /**
   * @brief Calculate the Coriolis and centripetal force matrix.
   *
   * @note The Coriolis and centripetal force matrix `C` is the sum of the rigid body Coriolis
   * forces and the added Coriolis forces such that `C = C_RB + C_A`.
   *
   * @param velocity The current velocity of the vehicle in the body frame.
   * @return Eigen::MatrixXd
   */
  [[nodiscard]] Eigen::MatrixXd calculateCoriolisMatrix(
    const geometry_msgs::msg::TwistStamped & velocity) const;

  /**
   * @brief Calculate the rigid body Coriolis-centripetal matrix.
   *
   * @note The definition of the rigid body Coriolis-centripetal matrix `C_RB` used in this work is
   * provided by Thor I. Fossen's textbook "Handbook of Marine Craft Hydrodynamics and Motion
   * Control" in Equation 3.57. Note that, in this model, we define the body frame to be coincident
   * with the center of mass, such that r_g^b = 0.
   *
   * @param mass The total mass of the vehicle (kg).
   * @param moments The moments of inertia.
   * @param velocity The current velocity of the vehicle in the body frame.
   * @return Eigen::MatrixXd
   */
  [[nodiscard]] static Eigen::MatrixXd calculateRigidBodyCoriolisMatrix(
    double mass, const MomentsOfInertia & moments,
    const geometry_msgs::msg::TwistStamped & velocity);

  /**
   * @brief Calculate the added Coriolis-centripetal matrix.
   *
   * @note The definition of the added Coriolis-centripetal matrix `C_A` used in this work is
   * provided by Gianluca Antonelli's "Underwater Robots" in Section 2.4.1.
   *
   * @param added_mass The added mass coefficients.
   * @param velocity The current velocity of the vehicle in the body frame.
   * @return Eigen::MatrixXd
   */
  [[nodiscard]] static Eigen::MatrixXd calculateAddedCoriolixMatrix(
    const AddedMass & added_mass, const geometry_msgs::msg::TwistStamped & velocity);

  /**
   * @brief Calculate the rigid body damping matrix.
   *
   * @note The rigid body damping matrix `D` is defined as the sum of the linear and nonlinear
   * damping coefficients.
   *
   * @param velocity The current velocity of the vehicle in the body frame.
   * @return Eigen::MatrixXd
   */
  [[nodiscard]] Eigen::MatrixXd calculateDampingMatrix(
    const geometry_msgs::msg::TwistStamped & velocity) const;

  /**
   * @brief Calculate the linear damping matrix.
   *
   * @note The definition of the linear damping matrix used in this work is provided by Gianluca
   * Antonelli's "Underwater Robots" in Section 2.4.2.
   *
   * @param linear_damping The linear damping coefficients.
   * @return Eigen::MatrixXd
   */
  [[nodiscard]] static Eigen::MatrixXd calculateLinearDampingMatrix(
    const LinearDamping & linear_damping);

  /**
   * @brief Calculate the nonlinear damping matrix.
   *
   * @note The definition of the nonlinear damping matrix used in this work is provided by Gianluca
   * Antonelli's "Underwater Robots" in Section 2.4.2.
   *
   * @param quadratic_damping The nonlinear damping coefficients.
   * @param velocity The current velocity of the vehicle in the body frame.
   * @return Eigen::MatrixXd
   */
  [[nodiscard]] static Eigen::MatrixXd calculateNonlinearDampingMatrix(
    const NonlinearDamping & quadratic_damping, const geometry_msgs::msg::TwistStamped & velocity);

  /**
   * @brief Calculate the vector of force/moment due to gravity and buoyancy in the body-fixed
   * frame.
   *
   * @note The definition used for the gravity and buoyancy vector `g` is given by Gianluca
   * Antonelli's "Underwater Robots" in Section 2.5.
   *
   * @param pose The current pose of the vehicle in the inertial frame.
   * @return Eigen::VectorXd
   */
  [[nodiscard]] Eigen::VectorXd calculateRestoringForcesVector(
    const geometry_msgs::msg::PoseStamped & pose) const;
};

}  // namespace blue::dynamics
