cmake_minimum_required(VERSION 3.8)
project(blue_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/blue_bringup)

ament_package()
