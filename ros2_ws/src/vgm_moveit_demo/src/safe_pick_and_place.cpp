#include <chrono>
#include <algorithm>
#include "vgm_safety_config.hpp"
#include <atomic>
#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace
{
using namespace vgm_safety;

bool finite_and_bounded(double x, double y, double z)
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
         x >= kWorkspaceLower[0] && x <= kWorkspaceUpper[0] &&
         y >= kWorkspaceLower[1] && y <= kWorkspaceUpper[1] &&
         z >= kWorkspaceLower[2] && z <= kWorkspaceUpper[2];
}

moveit_msgs::msg::CollisionObject make_box(
  const std::string& id, const std::array<double, 3>& position,
  const std::array<double, 3>& dimensions)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "world";
  object.pose.orientation.w = 1.0;
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
    stop_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "/vgm/stop", [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        cancel("operator_stop");
        response->success = true;
        response->message = "cancellation requested; no following primitives permitted";
      });
    watchdog_ = node_->create_wall_timer(std::chrono::milliseconds(50), [this]() {
      if (std::chrono::steady_clock::now() > deadline_) {
        cancel("execution_timeout");
      }
    });
  }

  void cancel(const char* reason)
  {
    if (!stopped_.exchange(true)) {
      arm_.stop();
      hand_.stop();
      RCLCPP_WARN(node_->get_logger(), "VGM_EXECUTION_STOP reason=%s", reason);
    }
  }

  bool run(
    const std::string& request_id, const std::string& object_id,
    const std::string& target_id, const std::array<double, 3>& object_position,
    const std::array<double, 3>& target,
    const std::string& skill = "pick_and_place", const std::string& pose_name = "")
  {
    if (skill == "close_gripper") {
      RCLCPP_ERROR(node_->get_logger(), "Standalone close requires validated grasp context");
      return false;
    }
    const auto held = planning_scene_.getAttachedObjects();
    if ((!held.empty() && (skill != "place" || held.size() != 1 || held.count(object_id) != 1)) ||
        (skill == "place" && held.count(object_id) != 1)) {
      RCLCPP_ERROR(node_->get_logger(), "Skill violates held-object preconditions");
      return false;
    }
    if (!add_static_collision_objects(object_id)) {
      return false;
    }
    if (skill == "move_named_pose") {
      return arm_.setNamedTarget(pose_name) && plan_and_execute(arm_, "move_named_pose");
    }
    if (skill == "open_gripper" || skill == "close_gripper") {
      return move_hand(skill == "open_gripper" ? "open" : "close", skill);
    }

    if (skill != "place") {
      if (!move_hand("open", "open_gripper")) {
        return false;
      }
      if (!move_arm(object_position[0], object_position[1], object_position[2] + k_pick_approach_height_m, "pick_approach")) {
        return false;
      }
      if (!move_arm(object_position[0], object_position[1], object_position[2] + k_pick_grasp_height_offset_m, "pick_descend")) {
        return false;
      }
      if (!move_hand("close", "close_gripper")) {
        return false;
      }
      if (!attach_object(object_id, object_position)) {
        return false;
      }
      if (!move_arm(object_position[0], object_position[1], object_position[2] + k_pick_retreat_height_m, "pick_retreat")) {
        return false;
      }
    }
    if (skill == "pick") {
      return !stopped_;
    }
    if (planning_scene_.getAttachedObjects({object_id}).count(object_id) != 1) {
      RCLCPP_ERROR(node_->get_logger(), "Placement requires the requested attached object");
      return false;
    }
    if (!move_arm(target[0], target[1], target[2] + k_place_approach_height_m, "place_approach")) {
      return false;
    }
    if (!move_arm(target[0], target[1], target[2] + k_place_release_height_offset_m, "place_descend")) {
      return false;
    }
    if (!move_hand("open", "release_gripper")) {
      return false;
    }
    if (!wait_for_open_feedback()) {
      return false;
    }
    if (!before_deadline("detach_object")) {
      return false;
    }
    if (!arm_.detachObject(object_id)) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to detach allowlisted object '%s'", object_id.c_str());
      return false;
    }
    // detachObject publishes asynchronously. Wait for the world transfer and
    // verify it before planning retreat; never remove the released obstacle.
    const auto detach_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (planning_scene_.getAttachedObjects({object_id}).count(object_id) != 0) {
      if (!before_deadline("detach_wait") || std::chrono::steady_clock::now() > detach_deadline) {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    const auto released = planning_scene_.getObjects({object_id});
    if (released.count(object_id) != 1 || released.at(object_id).primitives.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "Released object missing from collision world");
      return false;
    }
    // Preserve MoveIt's transformed release pose (not the spawn pose). Extend
    // downward to cover gravity settling to the measured support plane too.
    const auto& released_object = released.at(object_id);
    if (released_object.header.frame_id != "world" || released_object.primitive_poses.size() != 1 ||
        released_object.primitives.size() != 1) {return false;}
    const auto& box = released_object.primitives.at(0);
    const auto& pose = released_object.primitive_poses.at(0);
    if (box.type != shape_msgs::msg::SolidPrimitive::BOX || box.dimensions.size() != 3) {
      return false;
    }
    const auto& origin = released_object.pose;
    Eigen::Quaterniond rotation(origin.orientation.w, origin.orientation.x, origin.orientation.y, origin.orientation.z);
    Eigen::Quaterniond shape_rotation(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    if (!rotation.coeffs().allFinite() || !shape_rotation.coeffs().allFinite() ||
        rotation.norm() < 0.99 || shape_rotation.norm() < 0.99) {return false;}
    rotation.normalize();
    shape_rotation.normalize();
    const Eigen::Vector3d center = rotation * Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z) +
      Eigen::Vector3d(origin.position.x, origin.position.y, origin.position.z);
    Eigen::Vector3d dimensions = (rotation * shape_rotation).toRotationMatrix().cwiseAbs() *
      Eigen::Vector3d(box.dimensions[0], box.dimensions[1], box.dimensions[2]);
    if (!center.allFinite() || !dimensions.allFinite() || (dimensions.array() <= 0).any()) {return false;}
    const double placed_center_z = target[2] + kObjectSizes.at(object_id) / 2.0;
    const double release_z = center.z();
    if (!std::isfinite(release_z) || release_z < placed_center_z - kAttachedObjectClearance) {
      return false;
    }
    const double settling_distance = std::max(0.0, release_z - placed_center_z);
    dimensions.z() += settling_distance;
    const auto placed_obstacle = make_box(object_id,
      {center.x(), center.y(), center.z() - settling_distance / 2.0},
      {dimensions.x(), dimensions.y(), dimensions.z()});
    if (!planning_scene_.applyCollisionObject(placed_obstacle)) {
      return false;
    }
    RCLCPP_INFO(node_->get_logger(), "VGM_PRIMITIVE_COMPLETE kind=detach_object");
    if (!move_arm(target[0], target[1], target[2] + k_place_retreat_height_m, "place_retreat")) {
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
    group.setNumPlanningAttempts(kPlanningAttempts);
    group.setMaxVelocityScalingFactor(kVelocityScale);
    group.setMaxAccelerationScalingFactor(kAccelerationScale);
  }

  bool before_deadline(const std::string& primitive)
  {
    if (!motion_started_) {
      const auto now_s = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
      const auto age = now_s - node_->get_parameter("scene_captured_at_s").as_double();
      if (!std::isfinite(age) || age < 0 || age > kSceneAgeSeconds) {
        cancel("stale_scene_before_first_motion");
        return false;
      }
    }
    // Operator-only acceptance fixture. It can only inhibit motion.
    if (node_->has_parameter("fault_before_primitive") && primitive == "pick_approach" &&
        node_->get_parameter("fault_before_primitive").as_string() == "pick_approach") {
      RCLCPP_ERROR(node_->get_logger(), "VGM_FAULT_INJECTED before=pick_approach");
      cancel("injected_backend_fault");
      return false;
    }
    if (!stopped_ && rclcpp::ok() && std::chrono::steady_clock::now() <= deadline_) {
      return true;
    }
    cancel("stop_or_deadline");
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
    if (primitive == "place_retreat") {
      // Use the observed open-hand state, never the hand action's completion
      // timestamp as proof that the arm's state monitor has caught up.
      if (!wait_for_open_feedback()) {return false;}
      group.setStartState(*release_state_);
    } else {
      group.setStartStateToCurrentState();
    }
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      cancel("planning_failure");
      RCLCPP_ERROR(node_->get_logger(), "MoveIt planning failed for '%s'", primitive.c_str());
      return false;
    }
    if (!before_deadline(primitive)) {
      return false;
    }
    RCLCPP_INFO(node_->get_logger(), "VGM_PRIMITIVE_START kind=%s", primitive.c_str());
    motion_started_ = true;
    if (group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS || stopped_) {
      cancel("execution_failure");
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

  moveit::core::RobotStatePtr release_state_;

  bool wait_for_open_feedback()
  {
    const auto expected = hand_.getNamedTargetValues("open");
    const auto start = std::chrono::steady_clock::now();
    auto stable_since = start;
    bool stable = false;
    release_state_.reset();
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() <
      k_place_open_feedback_timeout_s) {
      if (!before_deadline("release_feedback")) {return false;}
      auto observed = arm_.getCurrentState(0.1);
      bool matches = observed && !expected.empty();
      for (const auto& [joint, value] : expected) {
        if (!observed) {matches = false; break;}
        const auto& names = observed->getRobotModel()->getVariableNames();
        if (std::find(names.begin(), names.end(), joint) == names.end()) {matches = false; break;}
        const double actual = observed->getVariablePosition(joint);
        matches = matches && std::isfinite(actual) &&
          std::abs(actual - value) <= k_place_open_feedback_tolerance_m;
      }
      const auto now = std::chrono::steady_clock::now();
      if (!matches) {stable = false;}
      else {
        if (!stable) {stable_since = now; stable = true;}
        if (std::chrono::duration<double>(now - stable_since).count() >=
          k_place_open_feedback_settle_s) {
          release_state_ = observed;
          RCLCPP_INFO(node_->get_logger(), "VGM_RELEASE_OPEN_FEEDBACK verified=true");
          return true;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    cancel("release_feedback_timeout");
    return false;
  }

  bool move_hand(const std::string& target, const std::string& primitive)
  {
    if (!hand_.setNamedTarget(target)) {
      RCLCPP_ERROR(node_->get_logger(), "MoveIt does not define hand target '%s'", target.c_str());
      return false;
    }
    return plan_and_execute(hand_, primitive);
  }

  bool add_static_collision_objects(const std::string& selected_object)
  {
    std::vector<moveit_msgs::msg::CollisionObject> objects;
    objects.push_back(make_box("table", kTablePosition, kTableDimensions));
    for (const auto& id : kAllowedObjects) {
      if (id == selected_object) {continue;}
      const auto parameter_name = "scene_" + id;
      if (!node_->has_parameter(parameter_name)) {
        RCLCPP_ERROR(node_->get_logger(), "Missing collision evidence for %s", id.c_str());
        return false;
      }
      const auto observed = node_->get_parameter(parameter_name).as_double_array();
      if (observed.size() != 3 || !finite_and_bounded(observed[0], observed[1], observed[2])) {
        return false;
      }
      const std::array<double, 3> position = {observed[0], observed[1], observed[2]};
      const double size = kObjectSizes.at(id);
      if (id != selected_object) {
        objects.push_back(make_box(id, position, {size, size, size}));
      }
    }
    return planning_scene_.applyCollisionObjects(objects);
  }

  bool attach_object(
    const std::string& object_id, const std::array<double, 3>& object_position)
  {
    if (!before_deadline("attach_object")) {
      return false;
    }
    const double size = kObjectSizes.at(object_id);
    auto planning_position = object_position;
    planning_position[2] += kAttachedObjectClearance;
    if (!planning_scene_.applyCollisionObject(
      make_box(object_id, planning_position, {size, size, size}))) {
      return false;
    }
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
  std::atomic<bool> stopped_{false};
  bool motion_started_{false};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr watchdog_;
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
  auto skill = string_parameter("skill");
  if (skill.empty()) {skill = "pick_and_place";}
  const auto pose_name = string_parameter("pose_name");
  const std::array<double, 3> object_position = {
    double_parameter("object_x"),
    double_parameter("object_y"),
    double_parameter("object_z"),
  };

  const std::array<double, 3> target_position = {
    double_parameter("target_x"), double_parameter("target_y"), double_parameter("target_z")};
  const auto digest = string_parameter("safety_policy_digest");
  const auto captured_at = double_parameter("scene_captured_at_s");
  const auto now_s = std::chrono::duration<double>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  if (digest != kPolicyDigest || !std::isfinite(captured_at) ||
      now_s - captured_at < 0 || now_s - captured_at > kSceneAgeSeconds) {
    RCLCPP_ERROR(node->get_logger(), "Policy build mismatch or stale scene; rebuild/reobserve required");
    rclcpp::shutdown();
    return 2;
  }
  const bool grounded = skill == "pick" || skill == "place" || skill == "pick_and_place";
  const bool placing = skill == "place" || skill == "pick_and_place";
  if (request_id.empty() || request_id.size() > 64 || kAllowedSkills.count(skill) == 0 ||
      (grounded && kAllowedObjects.count(object_id) == 0) ||
      ((skill == "pick" || skill == "pick_and_place") &&
       !finite_and_bounded(object_position[0], object_position[1], object_position[2])) ||
      (placing && (kAllowedTargets.count(target_id) == 0 ||
       !finite_and_bounded(target_position[0], target_position[1], target_position[2]))) ||
      (skill == "move_named_pose" && kAllowedNamedPoses.count(pose_name) == 0)) {
    RCLCPP_ERROR(
      node->get_logger(), "Rejected request: invalid ID, allowlist entry, or grounded position");
    rclcpp::shutdown();
    return 2;
  }

  rclcpp::executors::SingleThreadedExecutor ros_executor;
  ros_executor.add_node(node);
  std::thread spin_thread([&ros_executor]() { ros_executor.spin(); });
  int exit_code = 1;
  std::unique_ptr<SafeExecutor> executor;
  try {
    executor = std::make_unique<SafeExecutor>(node);
    exit_code = executor->run(request_id, object_id, target_id, object_position, target_position, skill, pose_name) ? 0 : 1;
    if (exit_code != 0) {executor->cancel("skill_failure");}
    RCLCPP_INFO(node->get_logger(), "VGM_SKILL_RESULT skill=%s status=%s", skill.c_str(),
                exit_code == 0 ? "succeeded" : "failed");
  } catch (const std::exception& error) {
    if (executor) {executor->cancel("executor_exception");}
    RCLCPP_ERROR(node->get_logger(), "Safe pick-and-place failed: %s", error.what());
  }

  ros_executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  executor.reset();
  rclcpp::shutdown();
  return exit_code;
}
