/* COMP0250 Coursework 1 - Team 11
 *
 * 
 * Credits - https://github.com/elena-ecn/pick-and-place
 * https://github.com/robosac333/Franka_Panda_Moveit2_Pick_Place
 * https://github.com/omarrayyann/pick-and-place-franke
 */

#include <cw1_class.h>

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <thread>
#include <chrono>
#include <vector>
#include <cmath>
#include <array>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rmw/qos_profiles.h>

static constexpr double FTIP = 0.105;
static constexpr int MAX_ATTEMPTS = 4;

// Robot positioning

static geometry_msgs::msg::Pose
make_pose(double x, double y, double z, const geometry_msgs::msg::Quaternion &o)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x; p.position.y = y; p.position.z = z;
  p.orientation = o; return p;
}

static geometry_msgs::msg::Pose td_pose(double x, double y, double z)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x; p.position.y = y; p.position.z = z;
  p.orientation.x = 1; p.orientation.y = 0;
  p.orientation.z = 0; p.orientation.w = 0; return p;
}

static inline double ft2l8(double z) { return z + FTIP; }

// Logic for movement of the Franka 

static bool joint_move(
  moveit::planning_interface::MoveGroupInterface &m,
  const geometry_msgs::msg::Pose &t, const rclcpp::Logger &l,
  const std::string &d)
{
  m.setPoseTarget(t);
  for (int a = 1; a <= 5; ++a) {
    moveit::planning_interface::MoveGroupInterface::Plan p;
    if (m.plan(p) != moveit::core::MoveItErrorCode::SUCCESS) continue;
    if (m.execute(p) == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(l, "%s: OK", d.c_str()); return true; }
  }
  RCLCPP_ERROR(l, "%s: FAIL", d.c_str()); return false;
}

static bool cart_move(
  moveit::planning_interface::MoveGroupInterface &m,
  const geometry_msgs::msg::Pose &t, const rclcpp::Logger &l,
  const std::string &d)
{
  std::vector<geometry_msgs::msg::Pose> w = {t};
  moveit_msgs::msg::RobotTrajectory tr;
  double f = m.computeCartesianPath(w, 0.005, 0.0, tr);
  if (f >= 0.90 && m.execute(tr) == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(l, "%s: OK (%.0f%%)", d.c_str(), f*100); return true; }
  return joint_move(m, t, l, d);
}

// Gripping Logic

static bool open_gripper(const rclcpp::Node::SharedPtr &n, const rclcpp::Logger &l)
{
  moveit::planning_interface::MoveGroupInterface h(n, "hand");
  h.setNamedTarget("open");
  moveit::planning_interface::MoveGroupInterface::Plan p;
  if (h.plan(p) != moveit::core::MoveItErrorCode::SUCCESS) return false;
  if (h.execute(p) != moveit::core::MoveItErrorCode::SUCCESS) return false;
  RCLCPP_INFO(l, "Gripper is now open"); return true;
}

static void strong_grip(const rclcpp::Node::SharedPtr &n, const rclcpp::Logger &l)
{
  moveit::planning_interface::MoveGroupInterface h(n, "hand");
  h.setMaxVelocityScalingFactor(1.0);
  h.setMaxAccelerationScalingFactor(1.0);
  
  
  RCLCPP_INFO(l, " GRIP (j1=0.020)");
  h.setJointValueTarget("panda_finger_joint1", 0.020);
  
  moveit::planning_interface::MoveGroupInterface::Plan p;
  if (h.plan(p) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_WARN(l, " Grip plan failed"); return;
  }
  h.execute(p);
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  RCLCPP_INFO(l, " Grip and roce is being applied");
}

static bool go_home(
  moveit::planning_interface::MoveGroupInterface &m, const rclcpp::Logger &l)
{
  m.setNamedTarget("ready");
  for (int a = 1; a <= 5; ++a) {
    moveit::planning_interface::MoveGroupInterface::Plan p;
    if (m.plan(p) != moveit::core::MoveItErrorCode::SUCCESS) continue;
    if (m.execute(p) == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(l, "Home ok"); return true; } }
  RCLCPP_ERROR(l, "Home fail"); return false;
}

static bool align_wrist(
  moveit::planning_interface::MoveGroupInterface &m,
  double j7, const rclcpp::Logger &l)
{
  auto jv = m.getCurrentJointValues();
  if (jv.size() < 7) return false;
  RCLCPP_INFO(l, "Wrist: %.4f -> %.4f", jv[6], j7);
  jv[6] = j7;
  m.setJointValueTarget(jv);
  moveit::planning_interface::MoveGroupInterface::Plan p;
  if (m.plan(p) != moveit::core::MoveItErrorCode::SUCCESS) return false;
  if (m.execute(p) != moveit::core::MoveItErrorCode::SUCCESS) return false;
  RCLCPP_INFO(l, "Wrist aligned"); return true;
}


cw1::cw1(const rclcpp::Node::SharedPtr &node)
{
  node_ = node;
  service_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  sensor_cb_group_  = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  t1_service_ = node_->create_service<cw1_world_spawner::srv::Task1Service>(
    "/task1_start", std::bind(&cw1::t1_callback, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, service_cb_group_);
  t2_service_ = node_->create_service<cw1_world_spawner::srv::Task2Service>(
    "/task2_start", std::bind(&cw1::t2_callback, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, service_cb_group_);
  t3_service_ = node_->create_service<cw1_world_spawner::srv::Task3Service>(
    "/task3_start", std::bind(&cw1::t3_callback, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, service_cb_group_);

  rclcpp::SubscriptionOptions jo; jo.callback_group = sensor_cb_group_;
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::QoS(50).reliable(),
    [this](const sensor_msgs::msg::JointState::ConstSharedPtr) {
      joint_state_msg_count_.fetch_add(1, std::memory_order_relaxed); }, jo);

  rclcpp::SubscriptionOptions co; co.callback_group = sensor_cb_group_;
  cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/r200/camera/depth_registered/points", rclcpp::QoS(10).reliable(),
    [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr) {
      cloud_msg_count_.fetch_add(1, std::memory_order_relaxed); }, co);

  node_->declare_parameter<bool>("use_gazebo_gui", true);
  enable_cloud_viewer_ = node_->declare_parameter<bool>("enable_cloud_viewer", false);
  move_home_on_start_ = node_->declare_parameter<bool>("move_home_on_start", false);
  use_path_constraints_ = node_->declare_parameter<bool>("use_path_constraints", false);
  use_cartesian_reach_ = node_->declare_parameter<bool>("use_cartesian_reach", false);
  allow_position_only_fallback_ = node_->declare_parameter<bool>("allow_position_only_fallback", allow_position_only_fallback_);
  cartesian_eef_step_ = node_->declare_parameter<double>("cartesian_eef_step", cartesian_eef_step_);
  cartesian_jump_threshold_ = node_->declare_parameter<double>("cartesian_jump_threshold", cartesian_jump_threshold_);
  cartesian_min_fraction_ = node_->declare_parameter<double>("cartesian_min_fraction", cartesian_min_fraction_);
  publish_programmatic_debug_ = node_->declare_parameter<bool>("publish_programmatic_debug", publish_programmatic_debug_);
  enable_task1_snap_ = node_->declare_parameter<bool>("enable_task1_snap", false);
  return_home_between_pick_place_ = node_->declare_parameter<bool>("return_home_between_pick_place", return_home_between_pick_place_);
  return_home_after_pick_place_ = node_->declare_parameter<bool>("return_home_after_pick_place", return_home_after_pick_place_);
  pick_offset_z_ = node_->declare_parameter<double>("pick_offset_z", pick_offset_z_);
  task3_pick_offset_z_ = node_->declare_parameter<double>("task3_pick_offset_z", task3_pick_offset_z_);
  task2_capture_enabled_ = node_->declare_parameter<bool>("task2_capture_enabled", task2_capture_enabled_);
  task2_capture_dir_ = node_->declare_parameter<std::string>("task2_capture_dir", task2_capture_dir_);
  place_offset_z_ = node_->declare_parameter<double>("place_offset_z", place_offset_z_);
  grasp_approach_offset_z_ = node_->declare_parameter<double>("grasp_approach_offset_z", grasp_approach_offset_z_);
  post_grasp_lift_z_ = node_->declare_parameter<double>("post_grasp_lift_z", post_grasp_lift_z_);
  gripper_grasp_width_ = node_->declare_parameter<double>("gripper_grasp_width", gripper_grasp_width_);
  joint_state_wait_timeout_sec_ = node_->declare_parameter<double>("joint_state_wait_timeout_sec", joint_state_wait_timeout_sec_);

  RCLCPP_INFO(node_->get_logger(), "cw1 class initialised");
}

// Use this function call Ishan and Chris for Tasks 2 AND 3

bool cw1::pick_and_place(const geometry_msgs::msg::Pose& obj_pose, const geometry_msgs::msg::Point& basket_loc)
{
  auto L = node_->get_logger();

  const double cx = obj_pose.position.x;
  const double cy = obj_pose.position.y;
  const double cz = obj_pose.position.z;
  const double bx = basket_loc.x;
  const double by = basket_loc.y;
  const double bz = basket_loc.z;

  moveit::planning_interface::MoveGroupInterface arm(node_, "panda_arm");
  arm.setPlanningTime(10.0);
  arm.setNumPlanningAttempts(10);
  arm.setMaxVelocityScalingFactor(0.5);
  arm.setMaxAccelerationScalingFactor(0.5);

  // Fixing issues with the gripper and floor tension
  const double grasp_l8   = ft2l8(cz + 0.005);
  const double transit_l8 = 0.40;
  const double release_l8 = ft2l8(bz + 0.10);
  const double pre_grasp_l8 = grasp_l8 + 0.085;

  auto fail = [&]() { open_gripper(node_, L); go_home(arm, L); return false; };

  go_home(arm, L);

  // Step by step instructions for the panda to complete the pick and place

  // 1. Moving above the cube and opening the gripper
  if (!joint_move(arm, td_pose(cx, cy, transit_l8), L, "Above")) return fail();
  if (!open_gripper(node_, L)) return fail();

  // 2. Moving to height to grasp the cube
  if (!cart_move(arm, make_pose(cx, cy, pre_grasp_l8, td_pose(0,0,0).orientation), L, "Pre-grasp high")) return fail();
  std::this_thread::sleep_for(std::chrono::milliseconds(400));

  // 3. Rotating wrist of franka is necessary
  if (!align_wrist(arm, M_PI_4, L)) return fail();   

  // 4. Getting the corrected pose
  auto ori = arm.getCurrentPose().pose.orientation;

  // 5. Descend and grip
  if (!cart_move(arm, make_pose(cx, cy, grasp_l8, ori), L, "Descend")) return fail();
  strong_grip(node_, L);

  // 6. Lift up
  if (!cart_move(arm, make_pose(cx, cy, transit_l8, ori), L, "Lift")) return fail();

  // 7. Move to basket and place cube
  if (!joint_move(arm, make_pose(bx, by, transit_l8, ori), L, "To basket")) return fail();
  if (!cart_move(arm, make_pose(bx, by, release_l8, ori), L, "Lower")) return fail();
  if (!open_gripper(node_, L)) return fail();
  
  // 8. Go back to original position when done
  cart_move(arm, make_pose(bx, by, transit_l8, ori), L, "Retreat");
  go_home(arm, L);
  
  return true; 
}

// Call back function for task 1 

void cw1::t1_callback(
  const std::shared_ptr<cw1_world_spawner::srv::Task1Service::Request> request,
  std::shared_ptr<cw1_world_spawner::srv::Task1Service::Response> response)
{
  (void)response;
  auto L = node_->get_logger();

  RCLCPP_INFO(L, "Task 1");
  RCLCPP_INFO(L, "Cube (%.4f,%.4f,%.4f) Basket (%.4f,%.4f,%.4f)", 
              request->object_loc.pose.position.x, request->object_loc.pose.position.y, request->object_loc.pose.position.z, 
              request->goal_loc.point.x, request->goal_loc.point.y, request->goal_loc.point.z);

  bool success = pick_and_place(request->object_loc.pose, request->goal_loc.point);

  if (success) {
    RCLCPP_INFO(L, "Task 1 complete");
  } else {
    RCLCPP_ERROR(L, "Task 1 error");
  }
}

void cw1::t2_callback(
  const std::shared_ptr<cw1_world_spawner::srv::Task2Service::Request> request,
  std::shared_ptr<cw1_world_spawner::srv::Task2Service::Response> response)
{ (void)request; (void)response; RCLCPP_INFO(node_->get_logger(), "Task 2 stub"); }

void cw1::t3_callback(
  const std::shared_ptr<cw1_world_spawner::srv::Task3Service::Request> request,
  std::shared_ptr<cw1_world_spawner::srv::Task3Service::Response> response)
{ (void)request; (void)response; RCLCPP_INFO(node_->get_logger(), "Task 3 stub"); }
