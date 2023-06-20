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

#include "blue_dynamics/thruster_dynamics.hpp"

#include <cmath>

namespace blue::dynamics
{

[[nodiscard]] inline std::tuple<int, int> calculateDeadZone(double voltage)
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

[[nodiscard]] inline std::tuple<int, int> calculateDeadZone() {}

[[nodiscard]] inline int calculatePwmFromThrustSurface(double force, double voltage)
{
  // Coefficients for the surface identified by fitting a surface to the thrust curves for the
  // voltages 10, 12, 14, 16, 18, 20 using Matlab's `fit` function with the `Poly23` surface
  const double p00 = 1439;
  const double p10 = 7.621;
  const double p01 = 42.06;
  const double p20 = -0.2692;
  const double p11 = -3.278;
  const double p02 = -0.1122;
  const double p21 = 0.08778;
  const double p12 = 0.006153;
  const double p03 = -0.001667;

  const double pwm = p00 + p10 * voltage + p01 * force + p20 * std::pow(voltage, 2) +
                     p11 * voltage * force + p02 * std::pow(force, 2) +
                     p21 * std::pow(voltage, 2) * force + p12 * voltage * std::pow(force, 2) +
                     p03 * std::pow(force, 3);

  return static_cast<int>(std::round(pwm));
}

[[nodiscard]] inline int calculatePwmFromThrustCurve(double force) {}

}  // namespace blue::dynamics
