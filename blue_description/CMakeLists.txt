cmake_minimum_required(VERSION 3.8)
project(blue_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config gazebo meshes description rviz
  DESTINATION share/blue_description
)

ament_package()
