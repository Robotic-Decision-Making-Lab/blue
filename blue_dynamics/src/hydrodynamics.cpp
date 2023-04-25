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

Hydrodynamics6d::Hydrodynamics6d(double x, double y, double z, double k, double m, double n)
: x(x),
  y(y),
  z(z),
  k(k),
  m(m),
  n(n)
{
}

[[nodiscard]] Eigen::VectorXd Hydrodynamics6d::toVector() const
{
  Eigen::VectorXd vec(6);  // NOLINT
  vec << x, y, z, k, m, n;

  return vec;
}

[[nodiscard]] Eigen::MatrixXd Hydrodynamics6d::toMatrix() const
{
  return toVector().asDiagonal().toDenseMatrix();
}

Hydrodynamics3d::Hydrodynamics3d(double x, double y, double z)
: x(x),
  y(y),
  z(z)
{
}

[[nodiscard]] Eigen::VectorXd Hydrodynamics3d::toVector() const
{
  Eigen::VectorXd vec(3);  // NOLINT
  vec << x, y, z;

  return vec;
}

[[nodiscard]] Eigen::MatrixXd Hydrodynamics3d::toMatrix() const
{
  return toVector().asDiagonal().toDenseMatrix();
}

MomentsOfInertia::MomentsOfInertia()
: Hydrodynamics3d(0.0, 0.0, 0.0)
{
}

MomentsOfInertia::MomentsOfInertia(double x, double y, double z)
: Hydrodynamics3d(x, y, z)
{
}

CenterOfBuoyancy::CenterOfBuoyancy()
: Hydrodynamics3d(0.0, 0.0, 0.0)
{
}

CenterOfBuoyancy::CenterOfBuoyancy(double x, double y, double z)
: Hydrodynamics3d(x, y, z)
{
}

CenterOfGravity::CenterOfGravity()
: Hydrodynamics3d(0.0, 0.0, 0.0)
{
}

CenterOfGravity::CenterOfGravity(double x, double y, double z)
: Hydrodynamics3d(x, y, z)
{
}

AddedMass::AddedMass()
: Hydrodynamics6d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
{
}

AddedMass::AddedMass(
  double x_u_dot, double y_v_dot, double z_w_dot, double k_p_dot, double m_q_dot, double n_r_dot)
: Hydrodynamics6d(x_u_dot, y_v_dot, z_w_dot, k_p_dot, m_q_dot, n_r_dot)
{
}

LinearDamping::LinearDamping()
: Hydrodynamics6d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
{
}

LinearDamping::LinearDamping(double x_u, double y_v, double z_w, double k_p, double m_q, double n_r)
: Hydrodynamics6d(x_u, y_v, z_w, k_p, m_q, n_r)
{
}

NonlinearDamping::NonlinearDamping()
: Hydrodynamics6d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
{
}

NonlinearDamping::NonlinearDamping(
  double x_uu, double y_vv, double z_ww, double k_pp, double m_qq, double n_rr)
: Hydrodynamics6d(x_uu, y_vv, z_ww, k_pp, m_qq, n_rr)
{
}

}  // namespace blue::dynamics
