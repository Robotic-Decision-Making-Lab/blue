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

#include "thruster_dynamics.hpp"

#include <cmath>

namespace blue::dynamics
{

[[nodiscard]] std::tuple<int, int> calculateDeadZone(double voltage)
{
  // The minimum PWM in the deadzone increases as the voltage increases
  // This is best represented by a 3rd order polynomial
  const int min_deadzone = static_cast<int>(std::round(
    0.00926 * std::pow(voltage, 3) - 0.488 * std::pow(voltage, 2) + 9.749 * voltage + 1401.84));

  // The maximum PWM in the deadzone decreases as the voltage increases
  // This is best represented by a 4th order polynomial
  const int max_deadzone = static_cast<int>(std::round(
    -0.005208 * std::pow(voltage, 4) + 0.3264 * std::pow(voltage, 3) -
    7.354 * std::pow(voltage, 2) + 69.087 * voltage + 1310.19));

  return std::tuple<int, int>(min_deadzone, max_deadzone);
}

[[nodiscard]] int calculatePwmFromThrustCurve(double force) {}

[[nodiscard]] double calculateForceFromThrustCurve(double force) {}

}  // namespace blue::dynamics
