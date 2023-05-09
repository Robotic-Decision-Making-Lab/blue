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

#include "blue_dynamics/thruster_dynamics.hpp"

namespace blue::dynamics::test
{

using blue::dynamics::calculateDeadZone;
using blue::dynamics::calculatePwmFromThrustSurface;

TEST(ThrusterDynamicsTest, TestDeadzoneModel)
{
  // These are measurements obtained from the BlueROV2 T200 characterization
  const int expected_min = 1476;
  const int expected_max = 1528;

  const double voltage = 20.0;

  const std::tuple<int, int> actual = calculateDeadZone(voltage);

  ASSERT_NEAR(expected_min, std::get<0>(actual), 1);
  ASSERT_NEAR(expected_max, std::get<1>(actual), 1);
}

TEST(ThrusterDynamicsTest, TestThrustSurfaceModel)
{
  // These are measurements obtained from the BlueROV2 T200 characterization
  const int expected_pwm = 1280;
  const double force = -1.86 * 9.8;  // Convert from KgF to N
  const double voltage = 20.0;

  const int actual = calculatePwmFromThrustSurface(force, voltage);

  ASSERT_NEAR(expected_pwm, actual, 15);
}

}  // namespace blue::dynamics::test

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();

  return result;
}
