cmake_minimum_required(VERSION 3.8)
project(my_robot_commander_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tl-expected REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(my_robot_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(test_moveit src/test_moveit.cpp)
ament_target_dependencies(test_moveit rclcpp moveit_ros_planning_interface)

add_executable(commander_template src/commander_template.cpp)
ament_target_dependencies(commander_template 
  rclcpp moveit_ros_planning_interface example_interfaces my_robot_interfaces)

add_executable(commander_combined src/commander_combined.cpp)
ament_target_dependencies(commander_combined 
  rclcpp moveit_ros_planning_interface example_interfaces my_robot_interfaces)

add_executable(pid_trajectory_adapter src/pid_trajectory_adapter.cpp)
ament_target_dependencies(
  pid_trajectory_adapter
  rclcpp
  rclcpp_action
  control_msgs
  example_interfaces
  sensor_msgs
)

install(TARGETS 
  test_moveit
  commander_template
  commander_combined
  pid_trajectory_adapter
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
