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
#include <vector>

/// @cond
namespace Eigen
{

// Extend the Eigen namespace to include commonly used matrix types
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

}  // namespace Eigen
/// @endcond

namespace blue::utility
{

/**
 * @brief Convert an std::vector into an Eigen::Matrix.
 *
 * @tparam T The type of values held by the vector.
 * @tparam major The order to copy over the elements in (e.g., ``Eigen::RowMajor``).
 * @param rows The number of rows in the resulting matrix.
 * @param cols The number of columns in the resulting matrix.
 * @param vec The vector to convert into a matrix.
 * @return The converted Eigen matrix.
 */
template <typename T, int major = Eigen::ColMajor>
inline Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> vectorToEigen(
  const std::vector<T> & vec, int rows, int cols)
{
  // While it would be preferable to define the rows and cols as template parameters, the
  // primary use-case for this method within the scope of this implementation is to use it with
  // ROS 2 parameters which are not always known at compile time (e.g., TCM). Therefore, the rows
  // and cols are made to be function parameters.
  typedef const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, major> M;
  Eigen::Map<M> mat(vec.data(), rows, cols);

  return mat;
}

}  // namespace blue::utility
