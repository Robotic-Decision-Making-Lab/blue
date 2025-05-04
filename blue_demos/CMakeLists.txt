cmake_minimum_required(VERSION 3.8)
project(blue_demos)

find_package(ament_cmake REQUIRED)

install(
    DIRECTORY
        control_integration/launch
        control_integration/config
        control_integration/description
    DESTINATION share/blue_demos/control_integration
)

install(
    DIRECTORY teleoperation/launch teleoperation/config
    DESTINATION share/blue_demos/teleoperation
)

install(
    DIRECTORY
        manipulator_systems/launch
        manipulator_systems/config
        manipulator_systems/description
    DESTINATION share/blue_demos/manipulator_systems
)

ament_package()
