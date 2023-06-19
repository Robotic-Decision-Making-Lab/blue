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

#pragma once

#include <Eigen/Dense>

namespace blue::dynamics
{

// Used to represent a 6x6 matrix: commonly used to represent hydrodynamic parameters.
using Matrix6d = Eigen::Matrix<double, 6, 6>;

// Used to represent a 6x1 matrix: commonly used to represent hydrodynamic parameters.
using Vector6d = Eigen::Matrix<double, 6, 1>;

/**
 * @brief Create a skew-symmetric matrix from the provided coefficients.
 *
 * @param a1 Coefficient one.
 * @param a2 Coefficient two.
 * @param a3 Coefficient three.
 * @return A skew-symmetric matrix.
 */
[[nodiscard]] Eigen::Matrix3d createSkewSymmetricMatrix(double a1, double a2, double a3);

/**
 * @brief Create a skew-symmetric matrix from the provided vector.
 *
 * @param vec The vector whose coefficients will be used to create the skew-symmetric matrix.
 * @return A skew-symmetric matrix.
 */
[[nodiscard]] Eigen::Matrix3d createSkewSymmetricMatrix(const Eigen::Vector3d & vec);

/**
 * @brief The inertia of the vehicle.
 */
class Inertia
{
public:
  /**
   * @brief Default constructor for the inertia object.
   */
  Inertia() = default;
  /**
   * @brief Construct a new Inertia object.
   *
   * @param mass The total mass of the vehicle (kg).
   * @param inertia_tensor_coeff The inertia tensor coefficients `(I_xx, I_yy, I_zz)` (kg m^2).
   * @param added_mass_coeff The added mass coefficients `(X_dot{u}, Y_dot{v}, Z_dot{w}, K_dot{p},
   *        M_dot{q}, N_dot{r})`.
   */
  Inertia(
    double mass, const Eigen::Vector3d & inertia_tensor_coeff, const Vector6d & added_mass_coeff);

  /**
   * @brief Get the vehicle's inertia matrix.
   *
   * @details The inertia matrix `M` is the sum of the rigid body mass `M_RB` and the added mass
   * `M_A` such that `M = M_RB + M_A`. The definition used for the rigid body inertia matrix `M_RB`
   * is provided by Thor I. Fossen's textbook "Handbook of Marine Craft Hydrodynamics and Motion
   * Control" in Equation 3.44. Note that, in this model, we define the body frame to be coincident
   * with the center of mass, such that r_g^b = 0. The result is that `M_RB` is a diagonal matrix.
   * The definition used for the added mass inertia matrix `M_A` is provided by Gianluca Antonelli's
   * textbook "Underwater Robots" in Section 2.4.1.
   *
   * @return The inertia matrix `M`.
   */
  [[nodiscard]] Matrix6d getInertia() const;

private:
  Matrix6d inertia_matrix_;
};

/**
 * @brief The Coriolis and centripetal forces acting on the vehicle.
 */
class Coriolis
{
public:
  /**
   * @brief Default constructor for the Coriolis object.
   */
  Coriolis() = default;

  /**
   * @brief Construct a new Coriolis object.
   *
   * @param mass The total mass of the vehicle (kg).
   * @param inertia_tensor_coeff The inertia tensor coefficients `(I_xx, I_yy, I_zz)` (kg m^2).
   * @param added_mass_coeff The added mass coefficients `(X_dot{u}, Y_dot{v}, Z_dot{w}, K_dot{p},
   *        M_dot{q}, N_dot{r})`.
   */
  Coriolis(
    double mass, const Eigen::Vector3d & inertia_tensor_coeff, const Vector6d & added_mass_coeff);

  /**
   * @brief Calculate the Coriolis and centripetal forces for the vehicle.
   *
   * @details The Coriolis and centripetal force matrix `C` is the sum of the rigid body Coriolis
   * forces `C_RB` and the added Coriolis forces `C_A` such that `C = C_RB + C_A`.
   *
   * @param velocity The current velocity of the vehicle in the body frame.
   * @return The Coriolis and centripetal force matrix `C`.
   */
  [[nodiscard]] Matrix6d calculateCoriolis(const Vector6d & velocity) const;

  /**
   * @brief Calculate the rigid body Coriolis matrix.
   *
   * @details The definition of the rigid body Coriolis-centripetal matrix `C_RB` used in this work
   * is provided by Thor I. Fossen's textbook "Handbook of Marine Craft Hydrodynamics and Motion
   * Control" in Equation 3.57. Note that, in this model, we define the body frame to be coincident
   * with the center of mass, such that r_g^b = 0.
   *
   * @note The preferred entrypoint for calculating the Coriolis matrix is through the
   * `calculateCoriolis` method; however, this method is made accessible for advanced users who wish
   * to calculate the rigid body Coriolis matrix directly (which may be necessary when calculating
   * the dynamics using the relative velocity).
   *
   * @param angular_velocity The current angular velocity of the vehicle in the body frame (rad/s).
   * @return The rigid body Coriolis matrix `C_RB`.
   */
  [[nodiscard]] Matrix6d calculateRigidBodyCoriolis(const Eigen::Vector3d & angular_velocity) const;

  /**
   * @brief Calculate the added Coriolis matrix.
   *
   * @details The definition of the added Coriolis-centripetal matrix `C_A` used in this work is
   * provided by Gianluca Antonelli's "Underwater Robots" in Section 2.4.1.
   *
   * @note The preferred entrypoint for calculating the Coriolis matrix is through the
   * `calculateCoriolis` method; however, this method is made accessible for advanced users who wish
   * to calculate the added Coriolis matrix directly (which may be necessary when calculating
   * the dynamics using the relative velocity).
   *
   * @param velocity The current velocity of the vehicle in the body frame.
   * @return The added Coriolis matrix `C_A`.
   */
  [[nodiscard]] Matrix6d calculateAddedCoriolis(const Vector6d & velocity) const;

private:
  double mass_{0.0};
  Eigen::Matrix3d moments_;
  Vector6d added_mass_coeff_;
};

/**
 * @brief The rigid-body damping forces acting on the vehicle.
 */
class Damping
{
public:
  /**
   * @brief Default constructor for the Damping object.
   */
  Damping() = default;

  /**
   * @brief Construct a new Damping object.
   *
   * @param linear_damping_coeff The linear damping coefficients `(X_u, Y_v, Z_w, K_p, M_q, N_r)`.
   * @param quadratic_damping_coeff The nonlinear damping coefficients `(X_u|u|, Y_v|v|, Z_w|w|,
   *        K_p|p|, M_q|q|, N_r|r|)`.
   */
  Damping(const Vector6d & linear_damping_coeff, const Vector6d & quadratic_damping_coeff);

  /**
   * @brief Calculate the damping forces for the vehicle.
   *
   * @details The rigid body damping matrix `D` is defined as the sum of the linear and nonlinear
   * damping coefficients. The definition of the linear and nonlinear damping matrices used in this
   * work is provided by Gianluca Antonelli's "Underwater Robots" in Section 2.4.2.
   *
   * @param velocity The current velocity of the vehicle in the body frame.
   * @return The damping matrix `D`.
   */
  [[nodiscard]] Matrix6d calculateDamping(const Vector6d & velocity) const;

private:
  Matrix6d linear_damping_;
  Vector6d quadratic_damping_coeff_;

  /**
   * @brief Calculate the nonlinear damping matrix.
   *
   * @param velocity The current velocity of the vehicle in the body frame.
   * @return The nonlinear damping matrix.
   */
  [[nodiscard]] Matrix6d calculateNonlinearDamping(const Vector6d & velocity) const;
};

/**
 * @brief The restoring forces acting on the vehicle.
 */
class RestoringForces
{
public:
  /**
   * @brief Default constructor for the RestoringForces object.
   */
  RestoringForces() = default;

  /**
   * @brief Construct a new RestoringForces object.
   *
   * @note The gravitational force vector `g` is given as [0, 0, 9.81]^T m/s^2.
   *
   * @param weight The weight of the vehicle (N). This should be defined according to the definition
   *        of `W` provided in Gianluca Antonelli's "Underwater Robots" in Section 2.5.
   * @param buoyancy The buoyancy force acting on the vehicle. This should be defined according to
   *        the definition of `B` provided in Gianluca Antonelli's "Underwater Robots" in
   *        Section 2.5.
   * @param center_of_buoyancy The center of buoyancy of the vehicle relative to the body frame.
   * @param center_of_gravity The center of gravity of the vehicle relative to the body frame.
   */
  RestoringForces(
    double weight, double buoyancy, const Eigen::Vector3d & center_of_buoyancy,
    const Eigen::Vector3d & center_of_gravity);

  /**
   * @brief Calculate the restoring forces for the vehicle.
   *
   * @param rotation The current rotation of the vehicle in the inertial frame.
   * @return The vector of restoring forces.
   */
  [[nodiscard]] Vector6d calculateRestoringForces(const Eigen::Matrix3d & rotation) const;

private:
  double weight_{0.0};
  double buoyancy_{0.0};
  Eigen::Vector3d center_of_buoyancy_;
  Eigen::Vector3d center_of_gravity_;
};

/**
 * @brief The velocity of the fluid in which the vehicle is operating.
 */
class CurrentEffects
{
public:
  /**
   * @brief Default constructor for the CurrentEffects object.
   */
  CurrentEffects() = default;

  /**
   * @brief Construct a new CurrentEffects object
   *
   * @param current_velocity The velocity of the fluid in the inertial frame.
   */
  explicit CurrentEffects(const Vector6d & current_velocity);

  /**
   * @brief Calculate the current in the body frame.
   *
   * @param rotation The current rotation of the vehicle in the inertial frame.
   * @return The velocity of the current in the body frame.
   */
  [[nodiscard]] Vector6d calculateCurrentEffects(const Eigen::Matrix3d & rotation) const;

private:
  Vector6d current_velocity_;
};

/**
 * @brief The hydrodynamic parameters for an underwater vehicle.
 *
 * @note This implementation draws from Thor I. Fossen's "Handbook of Marine Craft Hydrodynamics
 * and Motion Control" (2011) and Gianluca Antonelli's "Underwater Robots" (2014) for the
 * hydrodynamic model.
 *
 */
struct HydrodynamicParameters
{
  Inertia inertia;
  Coriolis coriolis;
  Damping damping;
  RestoringForces restoring_forces;
  CurrentEffects current_effects;

  HydrodynamicParameters() = default;

  HydrodynamicParameters(
    Inertia inertia, Coriolis coriolis, Damping damping, RestoringForces restoring_forces,
    CurrentEffects current_effects);
};

}  // namespace blue::dynamics
