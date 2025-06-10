cmake_minimum_required(VERSION 3.8)
project(blue_planner_cpp)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_lifecycle
  std_msgs
  nav_msgs
  trajectory_msgs
  tf2
  ompl
  Eigen3
  )


find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(ament_index_cpp REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(eigen3_cmake_module REQUIRED)
find_package(pinocchio REQUIRED)
# add_library(blue_planner_cpp SHARED
#   src/ompl_planner.cpp
# )

# target_include_directories(blue_planner_cpp
#   PUBLIC
#     $<INSTALL_INTERFACE:include/blue_planner_cpp>
#     $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
#     ${EIGEN3_INCLUDE_DIR}
#   PRIVATE
#     ${PROJECT_SOURCE_DIR}/src
# )
# target_compile_features(blue_planner_cpp PUBLIC cxx_std_17)
# target_link_libraries(blue_planner_cpp
#   PUBLIC
#     ${rclcpp_LIBRARIES}
# )

# target_compile_definitions(blue_planner_cpp PRIVATE "BLUE_PLANNER_CPP_BUILDING_DLL")
# ament_target_dependencies(blue_planner_cpp
#   PUBLIC
#     ${THIS_PACKAGE_INCLUDE_DEPENDS}
# )

include_directories(
  include
  /opt/ros/iron/include
  /opt/ros/iron/include/ompl-1.6
  ${EIGEN3_INCLUDE_DIRS}
  ${colcon_INCLUDE_DIRS}
)

add_executable(ompl_planner src/ompl_planner.cpp)

# target_compile_features(ompl_planner PUBLIC cxx_std_17)
# target_link_libraries(ompl_planner blue_planner_cpp)
target_link_libraries(ompl_planner PUBLIC ${OMPL_LIBRARIES})


ament_target_dependencies(ompl_planner PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# target_include_directories(ompl_planner PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(ompl_planner PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
add_executable(collision_check src/collision_check.cpp)
ament_target_dependencies(collision_check ament_index_cpp Eigen3 eigen3_cmake_module rclcpp std_msgs nav_msgs trajectory_msgs tf2 ompl rclcpp_lifecycle)
target_link_libraries(collision_check pinocchio::pinocchio)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_uncrustify_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  ompl_planner
  DESTINATION lib/blue_planner_cpp)

install(TARGETS
  collision_check
  DESTINATION lib/blue_planner_cpp)

install(
  DIRECTORY include/
  DESTINATION include/blue_planner_cpp
)
install(
  DIRECTORY launch
  DESTINATION share/blue_planner_cpp
)

# install(
#   TARGETS
#     blue_planner_cpp
#   EXPORT
#     export_blue_planner_cpp
#   RUNTIME DESTINATION bin
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   INCLUDES DESTINATION include
# )

# ament_export_targets(export_blue_planner_cpp HAS_LIBRARY_TARGET)
# ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
