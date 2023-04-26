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
 * @brief The thruster configuration matrix (TCM) for the BlueROV2.
 *
 * @note This is generally denoted as `B` in literature.
 */
const Eigen::MatrixXd kBlueRov2Tcm((Eigen::MatrixXd(6, 6) << 0.707, 0.707, -0.707, -0.707, 0.0, 0.0,
                                    0.707, -0.707, 0.707, -0.707, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                                    1.0, 0.051, -0.051, 0.051, -0.051, -0.111, 0.111, -0.051,
                                    -0.051, 0.051, 0.051, -0.002, -0.002, 0.167, -0.167, -0.175,
                                    0.175, 0.0, 0.0)
                                     .finished());

/**
 * @brief The thruster configuration matrix (TCM) for the BlueROV2 Heavy.
 *
 * @note This is generally denoted as `B` in literature.
 */
const Eigen::MatrixXd kBlueRov2HeavyTcm((Eigen::MatrixXd(6, 8) << 0.707, 0.707, -0.707, -0.707, 0.0,
                                         0.0, 0.0, 0.0, -0.707, 0.707, -0.707, 0.707, 0.0, 0.0, 0.0,
                                         0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0, 1.0, -1.0, 0.06, -0.06,
                                         0.06, -0.06, -0.218, -0.218, 0.218, 0.218, 0.06, 0.06,
                                         -0.06, -0.06, 0.120, -0.120, 0.120, -0.120, -0.1888,
                                         0.1888, 0.1888, -0.1888, 0.0, 0.0, 0.0, 0.0)
                                          .finished());

}  // namespace blue::dynamics
