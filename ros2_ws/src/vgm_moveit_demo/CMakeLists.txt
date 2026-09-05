cmake_minimum_required(VERSION 3.16)
project(vgm_moveit_demo)

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(VGM_CONFIG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../config")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${VGM_CONFIG_DIR}/safety_policy.json" "${VGM_CONFIG_DIR}/phase_1_scene.json"
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_safety_header.py"
  "${CMAKE_CURRENT_SOURCE_DIR}/../vgm_runtime/vgm_runtime/config.py")
execute_process(COMMAND ${Python3_EXECUTABLE}
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_safety_header.py"
  --config-directory "${VGM_CONFIG_DIR}" --output "${CMAKE_CURRENT_BINARY_DIR}/vgm_safety_config.hpp"
  COMMAND_ERROR_IS_FATAL ANY)

add_executable(safe_named_pose src/safe_named_pose.cpp)
target_include_directories(safe_named_pose PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")
target_compile_features(safe_named_pose PUBLIC cxx_std_17)
ament_target_dependencies(
  safe_named_pose
  moveit_ros_planning_interface
  rclcpp
)

add_executable(safe_pick_and_place src/safe_pick_and_place.cpp)
target_include_directories(safe_pick_and_place PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")
target_compile_features(safe_pick_and_place PUBLIC cxx_std_17)
ament_target_dependencies(
  safe_pick_and_place
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  shape_msgs
  std_srvs
)

install(TARGETS safe_named_pose safe_pick_and_place DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
