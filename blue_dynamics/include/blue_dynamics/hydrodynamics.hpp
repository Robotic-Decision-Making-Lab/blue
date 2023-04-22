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

namespace dynamics
{

/**
 * @brief
 *
 */
struct HydrodynamicsBase
{
  double x;
  double y;
  double z;
  double k;
  double m;
  double n;

  /**
   * @brief Construct a new Hydrodynamics Base object
   *
   * @param x
   * @param y
   * @param z
   * @param k
   * @param m
   * @param n
   */
  HydrodynamicsBase(double x, double y, double z, double k, double m, double n)
  : x(x),
    y(y),
    z(z),
    k(k),
    m(m),
    n(n)
  {
  }
};

/**
 * @brief
 *
 */
struct MomentsOfInertia
{
  double x;
  double y;
  double z;

  /**
   * @brief Construct a new Moments Of Inertia object
   *
   * @param x
   * @param y
   * @param z
   */
  MomentsOfInertia(double x, double y, double z)
  : x(x),
    y(y),
    z(z)
  {
  }
};

/**
 * @brief
 *
 */
struct AddedMass : HydrodynamicsBase
{
  /**
   * @brief Construct a new Added Mass object
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
  : HydrodynamicsBase(x_u_dot, y_v_dot, z_w_dot, k_p_dot, m_q_dot, n_r_dot)
  {
  }
};

/**
 * @brief
 *
 */
struct LinearDamping : HydrodynamicsBase
{
  /**
   * @brief Construct a new Linear Damping object
   *
   * @param x_u
   * @param y_v
   * @param z_w
   * @param k_p
   * @param m_q
   * @param n_r
   */
  LinearDamping(double x_u, double y_v, double z_w, double k_p, double m_q, double n_r)
  : HydrodynamicsBase(x_u, y_v, z_w, k_p, m_q, n_r)
  {
  }
};

/**
 * @brief
 *
 */
struct NonlinearDamping : HydrodynamicsBase
{
  /**
   * @brief Construct a new Nonlinear Damping object
   *
   * @param x_uu
   * @param y_vv
   * @param z_ww
   * @param k_pp
   * @param m_qq
   * @param n_rr
   */
  NonlinearDamping(double x_uu, double y_vv, double z_ww, double k_pp, double m_qq, double n_rr)
  : HydrodynamicsBase(x_uu, y_vv, z_ww, k_pp, m_qq, n_rr)
  {
  }
};

}  // namespace dynamics
