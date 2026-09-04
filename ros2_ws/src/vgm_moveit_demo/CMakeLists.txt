cmake_minimum_required(VERSION 3.16)
project(vgm_moveit_demo)

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(shape_msgs REQUIRED)

add_executable(safe_named_pose src/safe_named_pose.cpp)
target_compile_features(safe_named_pose PUBLIC cxx_std_17)
ament_target_dependencies(
  safe_named_pose
  moveit_ros_planning_interface
  rclcpp
)

add_executable(safe_pick_and_place src/safe_pick_and_place.cpp)
target_compile_features(safe_pick_and_place PUBLIC cxx_std_17)
ament_target_dependencies(
  safe_pick_and_place
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  shape_msgs
)

install(TARGETS safe_named_pose safe_pick_and_place DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
