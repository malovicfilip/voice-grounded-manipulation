#include <chrono>
#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace
{
constexpr double kVelocityScale = 0.20;
constexpr double kAccelerationScale = 0.20;
constexpr double kPlanningTimeSeconds = 5.0;
constexpr double kExecutionDeadlineSeconds = 120.0;
constexpr double kCubeSize = 0.05;
constexpr double kAttachedObjectClearance = 0.005;
const std::set<std::string> kAllowedObjects = {
  "red_cube", "green_cube", "blue_cube", "yellow_cube", "magenta_cube", "cyan_cube"};
const std::map<std::string, std::array<double, 3>> kAllowedTargets = {
  {"blue_target", {0.0, 0.3, 0.75}},
  {"yellow_target", {0.0, -0.3, 0.75}},
};

bool finite_and_bounded(double x, double y, double z)
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
         x >= -0.55 && x <= 0.55 && y >= -0.35 && y <= 0.35 &&
         z >= 0.74 && z <= 1.35;
}

moveit_msgs::msg::CollisionObject make_box(
  const std::string& id, const std::array<double, 3>& position,
  const std::array<double, 3>& dimensions)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "world";
  object.id = id;
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions = {dimensions[0], dimensions[1], dimensions[2]};
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x = position[0];
  pose.position.y = position[1];
  pose.position.z = position[2];
  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}

class SafeExecutor
{
public:
  explicit SafeExecutor(const rclcpp::Node::SharedPtr& node)
  : node_(node), arm_(node, "panda_arm"), hand_(node, "hand"),
    deadline_(std::chrono::steady_clock::now() +
              std::chrono::milliseconds(static_cast<int>(kExecutionDeadlineSeconds * 1000.0)))
  {
    configure_group(arm_);
    configure_group(hand_);
    arm_.setPoseReferenceFrame("world");
  }

  bool run(
    const std::string& request_id, const std::string& object_id,
    const std::string& target_id, const std::array<double, 3>& object_position)
  {
    const auto target = kAllowedTargets.at(target_id);
    add_static_collision_objects(object_id);

    if (!move_hand("open", "open_gripper")) {
      return false;
    }
    if (!move_arm(object_position[0], object_position[1], object_position[2] + 0.20, "pick_approach")) {
      return false;
    }
    if (!move_arm(object_position[0], object_position[1], object_position[2] + 0.105, "pick_descend")) {
      return false;
    }
    if (!move_hand("close", "close_gripper")) {
      return false;
    }
    if (!attach_object(object_id, object_position)) {
      return false;
    }
    if (!move_arm(object_position[0], object_position[1], object_position[2] + 0.23, "pick_retreat")) {
      return false;
    }
    if (!move_arm(target[0], target[1], target[2] + 0.25, "place_approach")) {
      return false;
    }
    if (!move_arm(target[0], target[1], target[2] + 0.14, "place_descend")) {
      return false;
    }
    if (!move_hand("open", "release_gripper")) {
      return false;
    }
    if (!arm_.detachObject(object_id)) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to detach allowlisted object '%s'", object_id.c_str());
      return false;
    }
    moveit_msgs::msg::CollisionObject released_object;
    released_object.header.frame_id = "world";
    released_object.id = object_id;
    released_object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    if (!planning_scene_.applyCollisionObject(released_object)) {
      RCLCPP_ERROR(
        node_->get_logger(), "Failed to remove released object '%s' from the planning scene",
        object_id.c_str());
      return false;
    }
    RCLCPP_INFO(node_->get_logger(), "VGM_PRIMITIVE_COMPLETE kind=detach_object");
    if (!move_arm(target[0], target[1], target[2] + 0.28, "place_retreat")) {
      return false;
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "VGM_PICK_PLACE_RESULT request_id=%s object=%s target=%s plan=success execution=success",
      request_id.c_str(), object_id.c_str(), target_id.c_str());
    return true;
  }

private:
  void configure_group(moveit::planning_interface::MoveGroupInterface& group)
  {
    group.setPlanningTime(kPlanningTimeSeconds);
    group.setNumPlanningAttempts(3);
    group.setMaxVelocityScalingFactor(kVelocityScale);
    group.setMaxAccelerationScalingFactor(kAccelerationScale);
  }

  bool before_deadline(const std::string& primitive)
  {
    if (std::chrono::steady_clock::now() <= deadline_) {
      return true;
    }
    arm_.stop();
    hand_.stop();
    RCLCPP_ERROR(
      node_->get_logger(), "Execution deadline reached before '%s'", primitive.c_str());
    return false;
  }

  bool plan_and_execute(
    moveit::planning_interface::MoveGroupInterface& group,
    const std::string& primitive)
  {
    if (!before_deadline(primitive)) {
      return false;
    }
    group.setStartStateToCurrentState();
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "MoveIt planning failed for '%s'", primitive.c_str());
      return false;
    }
    if (!before_deadline(primitive)) {
      return false;
    }
    if (group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "MoveIt execution failed for '%s'", primitive.c_str());
      return false;
    }
    RCLCPP_INFO(node_->get_logger(), "VGM_PRIMITIVE_COMPLETE kind=%s", primitive.c_str());
    return true;
  }

  bool move_arm(double x, double y, double z, const std::string& primitive)
  {
    if (!finite_and_bounded(x, y, z)) {
      RCLCPP_ERROR(node_->get_logger(), "Rejected out-of-bounds Cartesian goal for '%s'", primitive.c_str());
      return false;
    }
    geometry_msgs::msg::Pose pose;
    pose.orientation.x = 1.0;
    pose.orientation.w = 0.0;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    arm_.clearPoseTargets();
    if (!arm_.setPoseTarget(pose)) {
      RCLCPP_ERROR(node_->get_logger(), "MoveIt rejected the pose target for '%s'", primitive.c_str());
      return false;
    }
    const bool succeeded = plan_and_execute(arm_, primitive);
    arm_.clearPoseTargets();
    return succeeded;
  }

  bool move_hand(const std::string& target, const std::string& primitive)
  {
    if (!hand_.setNamedTarget(target)) {
      RCLCPP_ERROR(node_->get_logger(), "MoveIt does not define hand target '%s'", target.c_str());
      return false;
    }
    return plan_and_execute(hand_, primitive);
  }

  void add_static_collision_objects(const std::string& selected_object)
  {
    std::vector<moveit_msgs::msg::CollisionObject> objects;
    objects.push_back(make_box("table", {0.0, 0.0, 0.725}, {1.2, 0.8, 0.05}));
    const std::map<std::string, std::array<double, 3>> initial_positions = {
      {"red_cube", {-0.1, -0.18, 0.775}},
      {"green_cube", {0.1, -0.18, 0.775}},
      {"blue_cube", {0.3, -0.18, 0.775}},
      {"yellow_cube", {-0.1, 0.18, 0.775}},
      {"magenta_cube", {0.1, 0.18, 0.775}},
      {"cyan_cube", {0.3, 0.18, 0.775}},
    };
    for (const auto& [id, position] : initial_positions) {
      if (id != selected_object) {
        objects.push_back(make_box(id, position, {kCubeSize, kCubeSize, kCubeSize}));
      }
    }
    planning_scene_.applyCollisionObjects(objects);
  }

  bool attach_object(
    const std::string& object_id, const std::array<double, 3>& object_position)
  {
    auto planning_position = object_position;
    planning_position[2] += kAttachedObjectClearance;
    planning_scene_.applyCollisionObject(
      make_box(object_id, planning_position, {kCubeSize, kCubeSize, kCubeSize}));
    const auto* hand_model = arm_.getRobotModel()->getJointModelGroup("hand");
    if (hand_model == nullptr) {
      RCLCPP_ERROR(node_->get_logger(), "MoveIt hand model is unavailable");
      return false;
    }
    const auto touch_links = hand_model->getLinkModelNames();
    if (!arm_.attachObject(object_id, arm_.getEndEffectorLink(), touch_links)) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to attach allowlisted object '%s'", object_id.c_str());
      return false;
    }
    RCLCPP_INFO(node_->get_logger(), "VGM_PRIMITIVE_COMPLETE kind=attach_object");
    return true;
  }

  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface arm_;
  moveit::planning_interface::MoveGroupInterface hand_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;
  std::chrono::steady_clock::time_point deadline_;
};
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
    "safe_pick_and_place",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  const auto string_parameter = [&node](const std::string& name) {
      std::string value;
      if (node->has_parameter(name)) {
        node->get_parameter(name, value);
        return value;
      }
      return node->declare_parameter<std::string>(name, "");
    };
  const auto double_parameter = [&node](const std::string& name) {
      double value = 0.0;
      if (node->has_parameter(name)) {
        node->get_parameter(name, value);
        return value;
      }
      return node->declare_parameter<double>(name, 0.0);
    };
  const auto request_id = string_parameter("request_id");
  const auto object_id = string_parameter("object_id");
  const auto target_id = string_parameter("target_id");
  const std::array<double, 3> object_position = {
    double_parameter("object_x"),
    double_parameter("object_y"),
    double_parameter("object_z"),
  };

  if (request_id.empty() || request_id.size() > 64 || kAllowedObjects.count(object_id) == 0 ||
      kAllowedTargets.count(target_id) == 0 ||
      !finite_and_bounded(object_position[0], object_position[1], object_position[2])) {
    RCLCPP_ERROR(
      node->get_logger(), "Rejected request: invalid ID, allowlist entry, or grounded position");
    rclcpp::shutdown();
    return 2;
  }

  rclcpp::executors::SingleThreadedExecutor ros_executor;
  ros_executor.add_node(node);
  std::thread spin_thread([&ros_executor]() { ros_executor.spin(); });
  int exit_code = 1;
  try {
    SafeExecutor executor(node);
    exit_code = executor.run(request_id, object_id, target_id, object_position) ? 0 : 1;
  } catch (const std::exception& error) {
    RCLCPP_ERROR(node->get_logger(), "Safe pick-and-place failed: %s", error.what());
  }

  ros_executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  rclcpp::shutdown();
  return exit_code;
}
