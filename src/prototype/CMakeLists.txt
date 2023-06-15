cmake_minimum_required(VERSION 3.8)
project(prototype)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

find_package(ros2_control REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(velocity_controllers REQUIRED)
find_package(std_srvs REQUIRED)
find_package(motor_control REQUIRED)
find_package(tf2_ros REQUIRED)

install(
  DIRECTORY
    launch
    worlds
    models
    urdf
    meshes
    scripts
    rviz
  DESTINATION
    share/${PROJECT_NAME}/
)

install(
  PROGRAMS
  scripts/move_laser.py 
  scripts/move_shaft.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()