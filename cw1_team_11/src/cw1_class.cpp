/* COMP0250 Coursework 1 - Team 11
 *
 * Task 1: Pick and place at given positions using MoveIt.
 * Tasks 2 & 3: Stubs (to be implemented).
 *
 * Key geometry notes:
 *   - The panda_arm group plans for panda_link8 (the flange).
 *   - From panda_link8 to the fingertip is ~0.105 m (hand + finger length).
 *   - Cube: 0.04 m side, centroid at ~0.029 m above ground.
 *   - Basket: 0.10 m side, centroid at ~0.020 m above ground, top at ~0.07 m.
 *   - All positions from the service are in "panda_link0" frame (= world).
 *
 * Approach strategy:
 *   1. Joint-space for large moves (fast, less precise)
 *   2. Cartesian for alignment and vertical moves (precise)
 *   3. Always return home on completion or failure (prevents jams)
 */

#include <cw1_class.h>

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <thread>
#include <chrono>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <rmw/qos_profiles.h>

///////////////////////////////////////////////////////////////////////////////
// Constants
///////////////////////////////////////////////////////////////////////////////

static constexpr double FINGER_TIP_OFFSET = 0.105;

///////////////////////////////////////////////////////////////////////////////
// Helpers
///////////////////////////////////////////////////////////////////////////////

static geometry_msgs::msg::Pose
make_top_down_pose(double x, double y, double link8_z)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = link8_z;
  p.orientation.x = 1.0;
  p.orientation.y = 0.0;
  p.orientation.z = 0.0;
  p.orientation.w = 0.0;
  return p;
}

static inline double fingertip_to_link8(double fingertip_z)
{
  return fingertip_z + FINGER_TIP_OFFSET;
}

/** Joint-space planning with retries. */
static bool move_to_pose(
  moveit::planning_interface::MoveGroupInterface &mg,
  const geometry_msgs::msg::Pose &target,
  const rclcpp::Logger &logger,
  const std::string &desc,
  int max_attempts = 5)
{
  mg.setPoseTarget(target);
  for (int a = 1; a <= max_attempts; ++a)
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (mg.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(logger, "%s: plan %d/%d failed", desc.c_str(), a, max_attempts);
      continue;
    }
    if (mg.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(logger, "%s: OK", desc.c_str());
      return true;
    }
    RCLCPP_WARN(logger, "%s: exec %d/%d failed", desc.c_str(), a, max_attempts);
  }
  RCLCPP_ERROR(logger, "%s: FAILED", desc.c_str());
  return false;
}

/** Cartesian straight-line with fallback to joint-space. */
static bool cartesian_move(
  moveit::planning_interface::MoveGroupInterface &mg,
  const geometry_msgs::msg::Pose &target,
  const rclcpp::Logger &logger,
  const std::string &desc,
  double eef_step = 0.005,
  double jump_thresh = 0.0,
  double min_fraction = 0.90)
{
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(target);

  moveit_msgs::msg::RobotTrajectory traj;
  double fraction = mg.computeCartesianPath(
    waypoints, eef_step, jump_thresh, traj);

  if (fraction >= min_fraction) {
    if (mg.execute(traj) == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(logger, "%s: Cartesian OK (%.0f%%)",
                  desc.c_str(), fraction * 100.0);
      return true;
    }
    RCLCPP_WARN(logger, "%s: Cartesian exec failed, fallback to joint",
                desc.c_str());
  } else {
    RCLCPP_WARN(logger, "%s: Cartesian %.0f%%, fallback to joint",
                desc.c_str(), fraction * 100.0);
  }
  return move_to_pose(mg, target, logger, desc);
}

/** Move the gripper to a given total width. */
static bool move_gripper(
  const rclcpp::Node::SharedPtr &node,
  double width,
  const rclcpp::Logger &logger,
  const std::string &desc)
{
  moveit::planning_interface::MoveGroupInterface hand(node, "hand");
  hand.setJointValueTarget("panda_finger_joint1", width / 2.0);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (hand.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "%s: gripper plan failed", desc.c_str());
    return false;
  }
  if (hand.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "%s: gripper exec failed", desc.c_str());
    return false;
  }
  RCLCPP_INFO(logger, "%s: width=%.4f OK", desc.c_str(), width);
  return true;
}

/**
 * Move the arm to its named "ready" home position.
 * This clears any stuck state and resets the arm to a known configuration.
 * We try multiple times because the arm may be in a difficult configuration.
 */
static bool move_to_home(
  moveit::planning_interface::MoveGroupInterface &mg,
  const rclcpp::Logger &logger)
{
  mg.setNamedTarget("ready");
  for (int a = 1; a <= 5; ++a) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (mg.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(logger, "Home: plan %d/5 failed", a);
      continue;
    }
    if (mg.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(logger, "Home: OK");
      return true;
    }
    RCLCPP_WARN(logger, "Home: exec %d/5 failed", a);
  }
  RCLCPP_ERROR(logger, "Home: FAILED — arm may be stuck");
  return false;
}

///////////////////////////////////////////////////////////////////////////////
// Constructor
///////////////////////////////////////////////////////////////////////////////

cw1::cw1(const rclcpp::Node::SharedPtr &node)
{
  node_ = node;
  service_cb_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  sensor_cb_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  t1_service_ = node_->create_service<cw1_world_spawner::srv::Task1Service>(
    "/task1_start",
    std::bind(&cw1::t1_callback, this,
              std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, service_cb_group_);
  t2_service_ = node_->create_service<cw1_world_spawner::srv::Task2Service>(
    "/task2_start",
    std::bind(&cw1::t2_callback, this,
              std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, service_cb_group_);
  t3_service_ = node_->create_service<cw1_world_spawner::srv::Task3Service>(
    "/task3_start",
    std::bind(&cw1::t3_callback, this,
              std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, service_cb_group_);

  rclcpp::SubscriptionOptions js_opts;
  js_opts.callback_group = sensor_cb_group_;
  auto js_qos = rclcpp::QoS(rclcpp::KeepLast(50)).reliable();
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", js_qos,
    [this](const sensor_msgs::msg::JointState::ConstSharedPtr msg) {
      (void)msg;
      joint_state_msg_count_.fetch_add(1, std::memory_order_relaxed);
    }, js_opts);

  rclcpp::SubscriptionOptions cl_opts;
  cl_opts.callback_group = sensor_cb_group_;
  auto cl_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/r200/camera/depth_registered/points", cl_qos,
    [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
      (void)msg;
      cloud_msg_count_.fetch_add(1, std::memory_order_relaxed);
    }, cl_opts);

  node_->declare_parameter<bool>("use_gazebo_gui", true);
  enable_cloud_viewer_ = node_->declare_parameter<bool>("enable_cloud_viewer", false);
  move_home_on_start_ = node_->declare_parameter<bool>("move_home_on_start", false);
  use_path_constraints_ = node_->declare_parameter<bool>("use_path_constraints", false);
  use_cartesian_reach_ = node_->declare_parameter<bool>("use_cartesian_reach", false);
  allow_position_only_fallback_ = node_->declare_parameter<bool>(
    "allow_position_only_fallback", allow_position_only_fallback_);
  cartesian_eef_step_ = node_->declare_parameter<double>(
    "cartesian_eef_step", cartesian_eef_step_);
  cartesian_jump_threshold_ = node_->declare_parameter<double>(
    "cartesian_jump_threshold", cartesian_jump_threshold_);
  cartesian_min_fraction_ = node_->declare_parameter<double>(
    "cartesian_min_fraction", cartesian_min_fraction_);
  publish_programmatic_debug_ = node_->declare_parameter<bool>(
    "publish_programmatic_debug", publish_programmatic_debug_);
  enable_task1_snap_ = node_->declare_parameter<bool>("enable_task1_snap", false);
  return_home_between_pick_place_ = node_->declare_parameter<bool>(
    "return_home_between_pick_place", return_home_between_pick_place_);
  return_home_after_pick_place_ = node_->declare_parameter<bool>(
    "return_home_after_pick_place", return_home_after_pick_place_);
  pick_offset_z_ = node_->declare_parameter<double>("pick_offset_z", pick_offset_z_);
  task3_pick_offset_z_ = node_->declare_parameter<double>(
    "task3_pick_offset_z", task3_pick_offset_z_);
  task2_capture_enabled_ = node_->declare_parameter<bool>(
    "task2_capture_enabled", task2_capture_enabled_);
  task2_capture_dir_ = node_->declare_parameter<std::string>(
    "task2_capture_dir", task2_capture_dir_);
  place_offset_z_ = node_->declare_parameter<double>("place_offset_z", place_offset_z_);
  grasp_approach_offset_z_ = node_->declare_parameter<double>(
    "grasp_approach_offset_z", grasp_approach_offset_z_);
  post_grasp_lift_z_ = node_->declare_parameter<double>(
    "post_grasp_lift_z", post_grasp_lift_z_);
  gripper_grasp_width_ = node_->declare_parameter<double>(
    "gripper_grasp_width", gripper_grasp_width_);
  joint_state_wait_timeout_sec_ = node_->declare_parameter<double>(
    "joint_state_wait_timeout_sec", joint_state_wait_timeout_sec_);

  RCLCPP_INFO(node_->get_logger(), "cw1 class initialised");
}

///////////////////////////////////////////////////////////////////////////////
// Task 1: Pick and Place
///////////////////////////////////////////////////////////////////////////////

void cw1::t1_callback(
  const std::shared_ptr<cw1_world_spawner::srv::Task1Service::Request> request,
  std::shared_ptr<cw1_world_spawner::srv::Task1Service::Response> response)
{
  (void)response;
  auto log = node_->get_logger();

  // ── Read positions ──────────────────────────────────────────────────────
  const double cx = request->object_loc.pose.position.x;
  const double cy = request->object_loc.pose.position.y;
  const double cz = request->object_loc.pose.position.z;

  const double bx = request->goal_loc.point.x;
  const double by = request->goal_loc.point.y;
  const double bz = request->goal_loc.point.z;

  RCLCPP_INFO(log, "=== Task 1 ===");
  RCLCPP_INFO(log, "Cube:   (%.3f, %.3f, %.3f)", cx, cy, cz);
  RCLCPP_INFO(log, "Basket: (%.3f, %.3f, %.3f)", bx, by, bz);

  // ── MoveIt setup ────────────────────────────────────────────────────────
  moveit::planning_interface::MoveGroupInterface arm(node_, "panda_arm");
  arm.setPlanningTime(10.0);
  arm.setNumPlanningAttempts(10);
  arm.setMaxVelocityScalingFactor(0.5);
  arm.setMaxAccelerationScalingFactor(0.5);
  arm.setGoalPositionTolerance(0.001);
  arm.setGoalOrientationTolerance(0.01);

  // ── Compute heights ─────────────────────────────────────────────────────
  const double grasp_l8    = fingertip_to_link8(cz - 0.005);
  const double hover_l8    = grasp_l8 + 0.08;
  const double approach_l8 = 0.35;
  const double transit_l8  = 0.40;
  const double release_l8  = fingertip_to_link8(bz + 0.05 + 0.05);

  RCLCPP_INFO(log, "Heights: grasp=%.3f hover=%.3f approach=%.3f release=%.3f",
              grasp_l8, hover_l8, approach_l8, release_l8);

  // ── Helper: go home and return (used on failure) ────────────────────────
  // We use a lambda so any early return still resets the arm.
  auto go_home_and_open = [&]() {
    move_gripper(node_, 0.07, log, "Cleanup-Open gripper");
    move_to_home(arm, log);
  };

  // ══════════════════════════════════════════════════════════════════════════
  //  STEP 0: Start from home position for a clean known state
  // ══════════════════════════════════════════════════════════════════════════
  RCLCPP_INFO(log, "Moving to home position...");
  move_to_home(arm, log);

  // ══════════════════════════════════════════════════════════════════════════
  //  PICK SEQUENCE
  // ══════════════════════════════════════════════════════════════════════════

  // 1. Open gripper
  if (!move_gripper(node_, 0.07, log, "1-Open")) {
    go_home_and_open(); return;
  }

  // 2. Joint-space to approach height above cube (fast, rough)
  if (!move_to_pose(arm,
      make_top_down_pose(cx, cy, approach_l8),
      log, "2-Approach")) {
    go_home_and_open(); return;
  }

  // 3. Cartesian to precise hover directly above cube (corrects XY)
  if (!cartesian_move(arm,
      make_top_down_pose(cx, cy, hover_l8),
      log, "3-Align")) {
    go_home_and_open(); return;
  }

  // 4. Cartesian straight down to grasp (pure Z motion)
  if (!cartesian_move(arm,
      make_top_down_pose(cx, cy, grasp_l8),
      log, "4-Descend")) {
    go_home_and_open(); return;
  }

  // 5. Close gripper
  if (!move_gripper(node_, gripper_grasp_width_, log, "5-Grasp")) {
    go_home_and_open(); return;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // 6. Cartesian lift straight up (no lateral drift while holding cube)
  if (!cartesian_move(arm,
      make_top_down_pose(cx, cy, transit_l8),
      log, "6-Lift")) {
    go_home_and_open(); return;
  }

  // ══════════════════════════════════════════════════════════════════════════
  //  PLACE SEQUENCE
  // ══════════════════════════════════════════════════════════════════════════

  // 7. Joint-space to above basket (large lateral move)
  if (!move_to_pose(arm,
      make_top_down_pose(bx, by, transit_l8),
      log, "7-Above basket")) {
    go_home_and_open(); return;
  }

  // 8. Cartesian lower to release height
  if (!cartesian_move(arm,
      make_top_down_pose(bx, by, release_l8),
      log, "8-Lower")) {
    go_home_and_open(); return;
  }

  // 9. Open gripper to release
  if (!move_gripper(node_, 0.07, log, "9-Release")) {
    go_home_and_open(); return;
  }

  // 10. Cartesian retreat upward
  cartesian_move(arm,
      make_top_down_pose(bx, by, transit_l8),
      log, "10-Retreat");

  // ══════════════════════════════════════════════════════════════════════════
  //  ALWAYS return home at the end
  // ══════════════════════════════════════════════════════════════════════════
  move_to_home(arm, log);

  RCLCPP_INFO(log, "=== Task 1 complete ===");
}

///////////////////////////////////////////////////////////////////////////////
// Task 2 (stub)
///////////////////////////////////////////////////////////////////////////////

void cw1::t2_callback(
  const std::shared_ptr<cw1_world_spawner::srv::Task2Service::Request> request,
  std::shared_ptr<cw1_world_spawner::srv::Task2Service::Response> response)
{
  (void)request;
  (void)response;
  RCLCPP_INFO(node_->get_logger(), "Task 2 stub");
}

///////////////////////////////////////////////////////////////////////////////
// Task 3 (stub)
///////////////////////////////////////////////////////////////////////////////

void cw1::t3_callback(
  const std::shared_ptr<cw1_world_spawner::srv::Task3Service::Request> request,
  std::shared_ptr<cw1_world_spawner::srv::Task3Service::Response> response)
{
  (void)request;
  (void)response;
  RCLCPP_INFO(node_->get_logger(), "Task 3 stub");
}
