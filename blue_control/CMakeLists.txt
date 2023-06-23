cmake_minimum_required(VERSION 3.8)
project(blue_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++ 17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  ament_cmake
  blue_dynamics
  blue_msgs
  nav_msgs
  sensor_msgs
  mavros_msgs
  std_srvs
  eigen3_cmake_module
  Eigen3
  tf2_eigen
  tf2_geometry_msgs
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

set(CUSTOM_CONTROLLER_HEADERS
  include/blue_control/ismc.hpp
)

set(CUSTOM_CONTROLLER_SOURCES
  src/ismc.cpp
)

include_directories(
  include
)

# Create a library for the base_controller class
add_library(base_controller include/blue_control/controller.hpp src/controller.cpp)
ament_target_dependencies(base_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Create a library for the custom controllers; link this to the base controller
add_library(custom_controllers ${CUSTOM_CONTROLLER_HEADERS} ${CUSTOM_CONTROLLER_SOURCES})
ament_target_dependencies(custom_controllers ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(custom_controllers base_controller)

# Add executables for the custom controllers
add_executable(ismc src/ismc.cpp)
ament_target_dependencies(ismc ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(ismc custom_controllers)

# Install
install(
  TARGETS base_controller custom_controllers ismc
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Run linters found in package.xml except the two below
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
