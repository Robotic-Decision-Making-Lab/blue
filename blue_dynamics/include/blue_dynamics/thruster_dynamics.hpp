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

#include <cstdint>
#include <tuple>

namespace blue::dynamics
{

/**
 * @brief The minimum PWM value that can be sent to the T200 thrusters.
 */
const uint16_t kMinPwm = 1100;

/**
 * @brief The maximum PWM value that can be sent to the T200 thrusters.
 */
const uint16_t kMaxPwm = 1900;

/**
 * @brief A standard PWM value for no thrust.
 *
 * @note This can be used when the thruster is not being used, or when the desired thrust falls
 * within the deadband zone.
 */
const uint16_t kNoThrustPwm = 1500;

/**
 * @brief Calculates the T200 deadzone band for a given voltage.
 *
 * @param voltage The current battery voltage.
 * @return The minimum and maximum PWM values for the deadzone band.
 */
[[nodiscard]] std::tuple<int, int> calculateDeadZone(double voltage);

/**
 * @brief Approximates the PWM input needed to achieve a desired force given the current battery
 * voltage.
 *
 * @note This is a simplified technique for calculating the PWM input. A more accurate method would
 * perform a characterization of the thruster dynamics. The issue with that solution, however, is
 * that there is no feedback from the BlueROV2 thrusters to use as the state variables for the
 * system.
 *
 * @param force The desired force (N).
 * @param voltage The current battery voltage (V).
 * @return The PWM input needed to achieve the desired force.
 */
[[nodiscard]] int calculatePwmFromThrustSurface(double force, double voltage);

}  // namespace blue::dynamics
