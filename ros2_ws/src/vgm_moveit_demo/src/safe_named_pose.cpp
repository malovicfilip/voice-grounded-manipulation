#include <chrono>
#include <memory>
#include <set>
#include <string>
#include <thread>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{
constexpr double kVelocityScale = 0.20;
constexpr double kAccelerationScale = 0.20;
const std::set<std::string> kAllowedTargets = { "ready", "extended", "transport" };
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
    "safe_named_pose",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  std::string target = "ready";
  if (node->has_parameter("target")) {
    node->get_parameter("target", target);
  } else {
    target = node->declare_parameter<std::string>("target", target);
  }
  if (kAllowedTargets.count(target) == 0) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Rejected target '%s'. Allowed high-level targets: ready, extended, transport",
      target.c_str());
    rclcpp::shutdown();
    return 2;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  int exit_code = 1;
  try {
    moveit::planning_interface::MoveGroupInterface move_group(node, "panda_arm");
    move_group.setPlanningTime(8.0);
    move_group.setNumPlanningAttempts(3);
    move_group.setMaxVelocityScalingFactor(kVelocityScale);
    move_group.setMaxAccelerationScalingFactor(kAccelerationScale);
    move_group.setStartStateToCurrentState();

    if (!move_group.setNamedTarget(target)) {
      RCLCPP_ERROR(node->get_logger(), "MoveIt does not define named target '%s'", target.c_str());
    } else {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const auto plan_result = move_group.plan(plan);
      if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "MoveIt planning failed for '%s'", target.c_str());
      } else {
        RCLCPP_INFO(
          node->get_logger(),
          "MoveIt plan accepted for '%s' at velocity_scale=%.2f acceleration_scale=%.2f",
          target.c_str(), kVelocityScale, kAccelerationScale);
        const auto execution_result = move_group.execute(plan);
        if (execution_result != moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_ERROR(node->get_logger(), "MoveIt execution failed for '%s'", target.c_str());
        } else {
          RCLCPP_INFO(
            node->get_logger(), "VGM_DEMO_RESULT target=%s plan=success execution=success", target.c_str());
          exit_code = 0;
        }
      }
    }
  } catch (const std::exception& error) {
    RCLCPP_ERROR(node->get_logger(), "MoveIt demo failed: %s", error.what());
  }

  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  rclcpp::shutdown();
  return exit_code;
}
