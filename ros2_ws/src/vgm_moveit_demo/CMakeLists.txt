cmake_minimum_required(VERSION 3.16)
project(vgm_moveit_demo)

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(safe_named_pose src/safe_named_pose.cpp)
target_compile_features(safe_named_pose PUBLIC cxx_std_17)
ament_target_dependencies(
  safe_named_pose
  moveit_ros_planning_interface
  rclcpp
)

install(TARGETS safe_named_pose DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
