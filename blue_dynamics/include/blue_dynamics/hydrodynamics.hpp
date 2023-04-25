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

/**
 * @brief A common base class to use for defining a collection of N-D hydrodynamics coefficients.
 */
struct HydrodynamicsXd
{
  /**
   * @brief Convert the coefficients into a column vector.
   *
   * @return The coefficients as a vector.
   */
  [[nodiscard]] virtual Eigen::VectorXd toVector() const = 0;

  /**
   * @brief Convert the coefficients into a matrix.
   *
   * @return The coefficients as a matrix.
   */
  [[nodiscard]] virtual Eigen::MatrixXd toMatrix() const = 0;
};

/**
 * @brief A base struct that can be used to define a collection of 6-D hydrodynamic coefficients.
 */
struct Hydrodynamics6d : HydrodynamicsXd
{
  double x;  // The X coefficient.
  double y;  // The Y coefficient.
  double z;  // The Z coefficient.
  double k;  // The K coefficient.
  double m;  // The M coefficient.
  double n;  // The Z coefficient.

  /**
   * @brief Construct a new Hydrodynamics6d object.
   *
   * @param x The X coefficient.
   * @param y The Y coefficient.
   * @param z The Z coefficient.
   * @param k The K coefficient.
   * @param m The M coefficient.
   * @param n The Z coefficient.
   */
  Hydrodynamics6d(double x, double y, double z, double k, double m, double n);

  /**
   * @brief Create a vector using the coefficients.
   *
   * @return The coefficients as a column vector.
   */
  [[nodiscard]] Eigen::VectorXd toVector() const override;

  /**
   * @brief Create a matrix using the coefficients.
   *
   * @note The base struct creates a diagonal matrix from the coefficients.
   *
   * @return The coefficients as a 6x6 diagonal matrix.
   */
  [[nodiscard]] Eigen::MatrixXd toMatrix() const override;
};

/**
 * @brief A base struct that can be used to define a collection of 3-D hydrodynamic coefficients.
 */
struct Hydrodynamics3d : HydrodynamicsXd
{
  double x;  // The X coefficient.
  double y;  // The Y coefficient.
  double z;  // The Z coefficient.

  /**
   * @brief Construct a new Hydrodynamics3d object
   *
   * @param x The X coefficient.
   * @param y The Y coefficient.
   * @param z The Z coefficient.
   */
  Hydrodynamics3d(double x, double y, double z);

  /**
   * @brief Create a vector from the coefficients.
   *
   * @return The coefficients as a column vector.
   */
  [[nodiscard]] Eigen::VectorXd toVector() const override;

  /**
   * @brief Create a matrix from the coefficients.
   *
   * @note The base struct creates a diagonal matrix from the coefficients.
   *
   * @return The coefficients as a 3x3 diagonal matrix.
   */
  [[nodiscard]] Eigen::MatrixXd toMatrix() const override;
};

/**
 * @brief The moments of inertia of a rigid body.
 */
struct MomentsOfInertia : Hydrodynamics3d
{
  /**
   * @brief Construct a new default MomentsOfInertia object.
   */
  MomentsOfInertia();

  /**
   * @brief Construct a new MomentsOfInertia object.
   *
   * @param x The I_xx coefficient.
   * @param y The I_yy coefficient.
   * @param z The I_zz coefficient.
   */
  MomentsOfInertia(double x, double y, double z);
};

/**
 * @brief The center of buoyancy of a submerged body.
 */
struct CenterOfBuoyancy : Hydrodynamics3d
{
  /**
   * @brief Construct a new default CenterOfBuoyancy object.
   */
  CenterOfBuoyancy();

  /**
   * @brief Construct a new CenterOfBuoyancy object.
   *
   * @param x The center of buoyancy along the x-axis.
   * @param y The center of buoyancy along the y-axis.
   * @param z The center of buoyancy along the z-axis.
   */
  CenterOfBuoyancy(double x, double y, double z);
};

/**
 * @brief The center of gravity of a submerged body.
 */
struct CenterOfGravity : Hydrodynamics3d
{
  /**
   * @brief Construct a new default CenterOfGravity object.
   */
  CenterOfGravity();

  /**
   * @brief Construct a new CenterOfGravity object.
   *
   * @param x The center of gravity along the x-axis.
   * @param y The center of gravity along the y-axis.
   * @param z The center of gravity along the z-axis.
   */
  CenterOfGravity(double x, double y, double z);
};

/**
 * @brief The added mass hydrodynamic coefficients.
 */
struct AddedMass : Hydrodynamics6d
{
  /**
   * @brief Construct a new default AddedMass object.
   */
  AddedMass();

  /**
   * @brief Construct a new AddedMass object.
   *
   * @param x_u_dot The X_dot{u} coefficient.
   * @param y_v_dot The X_dot{v} coefficient.
   * @param z_w_dot The Z_dot{w} coefficient.
   * @param k_p_dot The K_dot{p} coefficient.
   * @param m_q_dot The K_dot{q} coefficient.
   * @param n_r_dot The N_dot{r} coefficient.
   */
  AddedMass(
    double x_u_dot, double y_v_dot, double z_w_dot, double k_p_dot, double m_q_dot, double n_r_dot);
};

/**
 * @brief Linear damping coefficients.
 *
 */
struct LinearDamping : Hydrodynamics6d
{
  /**
   * @brief Construct a new default LinearDamping object.
   */
  LinearDamping();

  /**
   * @brief Construct a new LinearDamping object.
   *
   * @param x_u The X_u coefficient.
   * @param y_v The Y_v coefficient.
   * @param z_w The Z_w coefficient.
   * @param k_p The K_p coefficient.
   * @param m_q The M_q coefficient.
   * @param n_r The N_r coefficient.
   */
  LinearDamping(double x_u, double y_v, double z_w, double k_p, double m_q, double n_r);
};

/**
 * @brief Nonlinear damping coefficients.
 *
 */
struct NonlinearDamping : Hydrodynamics6d
{
  /**
   * @brief Construct a new default NonlinearDamping object.
   */
  NonlinearDamping();

  /**
   * @brief Construct a new NonlinearDamping object
   *
   * @param x_uu The X_{u|u|} coefficient.
   * @param y_vv The Y_{v|v|} coefficient.
   * @param z_ww The Z_{w|w|} coefficient.
   * @param k_pp The K_{p|p|} coefficient.
   * @param m_qq The M_{q|q|} coefficient.
   * @param n_rr The N_{r|r|} coefficient.
   */
  NonlinearDamping(double x_uu, double y_vv, double z_ww, double k_pp, double m_qq, double n_rr);
};

}  // namespace blue::dynamics
