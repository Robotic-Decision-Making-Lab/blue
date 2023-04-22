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

namespace blue::dynamics
{

struct Hydrodynamics6d
{
  double x;
  double y;
  double z;
  double k;
  double m;
  double n;

  Hydrodynamics6d(double x, double y, double z, double k, double m, double n)
  : x(x),
    y(y),
    z(z),
    k(k),
    m(m),
    n(n)
  {
  }

  [[nodiscard]] virtual Eigen::MatrixXd toMatrix() const
  {
    Eigen::VectorXd vec;
    vec << x, y, z, k, m, n;

    return vec.asDiagonal().toDenseMatrix();
  };
};

struct Hydrodynamics3d
{
  double x;
  double y;
  double z;

  Hydrodynamics3d(double x, double y, double z)
  : x(x),
    y(y),
    z(z)
  {
  }

  [[nodiscard]] virtual Eigen::Matrix3d toMatrix() const
  {
    Eigen::Vector3d vec;
    vec << x, y, z;

    return vec.asDiagonal().toDenseMatrix();
  };
};

struct MomentsOfInertia : Hydrodynamics3d
{
  MomentsOfInertia(double x, double y, double z)
  : Hydrodynamics3d(x, y, z)
  {
  }
};

struct CenterOfBuoyancy : Hydrodynamics3d
{
  CenterOfBuoyancy(double x, double y, double z)
  : Hydrodynamics3d(x, y, z)
  {
  }
};

struct AddedMass : Hydrodynamics6d
{
  /**
   * @brief Construct a new AddedMass object.
   *
   * @param x_u_dot
   * @param y_v_dot
   * @param z_w_dot
   * @param k_p_dot
   * @param m_q_dot
   * @param n_r_dot
   */
  AddedMass(
    double x_u_dot, double y_v_dot, double z_w_dot, double k_p_dot, double m_q_dot, double n_r_dot)
  : Hydrodynamics6d(x_u_dot, y_v_dot, z_w_dot, k_p_dot, m_q_dot, n_r_dot)
  {
  }
};

/**
 * @brief Linear damping coefficients.
 *
 */
struct LinearDamping : Hydrodynamics6d
{
  /**
   * @brief Construct a new LinearDamping object.
   *
   * @param x_u
   * @param y_v
   * @param z_w
   * @param k_p
   * @param m_q
   * @param n_r
   */
  LinearDamping(double x_u, double y_v, double z_w, double k_p, double m_q, double n_r)
  : Hydrodynamics6d(x_u, y_v, z_w, k_p, m_q, n_r)
  {
  }
};

/**
 * @brief Nonlinear damping coefficients.
 *
 */
struct NonlinearDamping : Hydrodynamics6d
{
  /**
   * @brief Construct a new NonlinearDamping object
   *
   * @param x_uu
   * @param y_vv
   * @param z_ww
   * @param k_pp
   * @param m_qq
   * @param n_rr
   */
  NonlinearDamping(double x_uu, double y_vv, double z_ww, double k_pp, double m_qq, double n_rr)
  : Hydrodynamics6d(x_uu, y_vv, z_ww, k_pp, m_qq, n_rr)
  {
  }
};

}  // namespace blue::dynamics
