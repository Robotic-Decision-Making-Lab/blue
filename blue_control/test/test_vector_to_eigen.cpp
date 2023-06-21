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

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <vector>

#include "blue_control/controller.hpp"

namespace blue::controller::test
{

using blue::control::convertVectorToEigenMatrix;

TEST(ControllerTest, TestVectorToEigenVector)
{
  const std::vector<int> coeff = {1, 2, 3, 4};

  Eigen::Matrix<int, 4, 1> expected;  // NOLINT
  expected << 1, 2, 3, 4;

  const Eigen::Matrix<int, 4, 1> actual = convertVectorToEigenMatrix<int>(coeff, 4, 1);

  ASSERT_TRUE(actual.isApprox(expected));
}

TEST(ControllerTest, TestVectorToEigenMatrix)
{
  const std::vector<int> coeff = {1, 2, 3, 4};

  Eigen::Matrix<int, 2, 2> expected;  // NOLINT
  expected << 1, 2, 3, 4;

  const Eigen::Matrix<int, 2, 2> actual =
    convertVectorToEigenMatrix<int, Eigen::RowMajor>(coeff, 2, 2);

  std::cout << expected << std::endl << actual << std::endl;

  ASSERT_TRUE(actual.isApprox(expected));
}

}  // namespace blue::controller::test

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();

  return result;
}
