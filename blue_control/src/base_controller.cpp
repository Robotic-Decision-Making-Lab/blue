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

#include "blue_control/base_controller.hpp"

namespace blue::control
{

Eigen::VectorXd convertVectorToEigenVector(const std::vector<double> & vec)
{
  Eigen::VectorXd eigen_vec(vec.size());
  eigen_vec = Eigen::Map<const Eigen::VectorXd>(vec.data(), vec.size());

  return eigen_vec;
}

Eigen::MatrixXd convertVectorToEigenMatrix(
  const std::vector<double> & vec, size_t rows, size_t cols)
{
  Eigen::Map<const Eigen::MatrixXd> mat(vec.data(), rows, cols);
  return mat;
}

BaseController::BaseController(const std::string & node_name)
: Node(std::move(node_name))
{
  // Declare ROS parameters
  this->declare_parameter("mass", 11.5);
  this->declare_parameter("buoyancy", 112.80);
  this->declare_parameter("weight", 114.80);
  this->declare_parameter("inertia_tensor_coeff", std::vector<double>{0.16, 0.16, 0.16});
  this->declare_parameter(
    "added_mass_coeff", std::vector<double>{-5.50, -12.70, -14.60, -0.12, -0.12, -0.12});
  this->declare_parameter(
    "linear_damping_coeff", std::vector<double>{-4.03, -6.22, -5.18, -0.07, -0.07, -0.07});
  this->declare_parameter(
    "quadratic_damping_coeff", std::vector<double>{-18.18, -21.66, -36.99, -1.55, -1.55, -1.55});
  this->declare_parameter("center_of_gravity", std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter("center_of_buoyancy", std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter("ocean_current", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  this->declare_parameter("num_thrusters", 8);

  // I'm so sorry for this
  // You can blame the ROS devs for not supporting nested arrays for parameters
  this->declare_parameter(
    "tcm", std::vector<double>{0.707,   0.707,  -0.707, -0.707,  0.0,    0.0,    0.0,   0.0,
                               -0.707,  0.707,  -0.707, 0.707,   0.0,    0.0,    0.0,   0.0,
                               0.0,     0.0,    0.0,    0.0,     -1.0,   1.0,    1.0,   -1.0,
                               0.06,    -0.06,  0.06,   -0.06,   -0.218, -0.218, 0.218, 0.218,
                               0.06,    0.06,   -0.06,  -0.06,   0.120,  -0.120, 0.120, -0.120,
                               -0.1888, 0.1888, 0.1888, -0.1888, 0.0,    0.0,    0.0,   0.0});

  // Get the parameter values
  const double mass = this->get_parameter("mass").as_double();
  const double buoyancy = this->get_parameter("buoyancy").as_double();
  const double weight = this->get_parameter("weight").as_double();
  const Eigen::Vector3d inertia_tensor_coeff =
    convertVectorToEigenVector(this->get_parameter("inertia_tensor_coeff").as_double_array());
  const Eigen::VectorXd added_mass_coeff =
    convertVectorToEigenVector(this->get_parameter("added_mass_coeff").as_double_array());
  const Eigen::VectorXd linear_damping_coeff =
    convertVectorToEigenVector(this->get_parameter("linear_damping_coeff").as_double_array());
  const Eigen::VectorXd quadratic_damping_coeff =
    convertVectorToEigenVector(this->get_parameter("quadratic_damping_coeff").as_double_array());
  const Eigen::Vector3d center_of_gravity =
    convertVectorToEigenVector(this->get_parameter("center_of_gravity").as_double_array());
  const Eigen::Vector3d center_of_buoyancy =
    convertVectorToEigenVector(this->get_parameter("center_of_buoyancy").as_double_array());
  const Eigen::VectorXd ocean_current =
    convertVectorToEigenVector(this->get_parameter("ocean_current").as_double_array());

  // Get the thruster configuration matrix
  std::vector<double> tcm_vec = this->get_parameter("tcm").as_double_array();
  size_t num_thrusters = this->get_parameter("num_thrusters").as_int();
  tcm_ = convertVectorToEigenMatrix(tcm_vec, tcm_vec.size() / num_thrusters, num_thrusters);

  // Initialize the hydrodynamic parameters
  hydrodynamics_ = blue::dynamics::HydrodynamicParameters(
    blue::dynamics::Inertia(mass, inertia_tensor_coeff, added_mass_coeff),
    blue::dynamics::Coriolis(mass, inertia_tensor_coeff, added_mass_coeff),
    blue::dynamics::Damping(linear_damping_coeff, quadratic_damping_coeff),
    blue::dynamics::RestoringForces(weight, buoyancy, center_of_buoyancy, center_of_gravity),
    blue::dynamics::CurrentEffects(ocean_current));

  rc_override_pub_ =
    this->create_publisher<mavros_msgs::msg::OverrideRCIn>("mavros/rc/override", 1);

  battery_state_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
    "/mavros/battery", 1,
    [this](sensor_msgs::msg::BatteryState::ConstSharedPtr msg) { battery_state_ = *msg; });

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/mavros/local_position/odom", 1,
    [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { odom_ = *msg; });

  // Run at a control rate of 200 Hz
  // ArduSub only runs at a rate of 100 Hz, but we want to make sure to run the controller at
  // a faster rate than the autopilot
  control_loop_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(5), [this]() { rc_override_pub_->publish(update()); });
}

}  // namespace blue::control
