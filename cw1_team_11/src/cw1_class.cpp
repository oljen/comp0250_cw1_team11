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


#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rmw/qos_profiles.h>

static constexpr double FTIP = 0.105;
static constexpr int MAX_ATTEMPTS = 4;


/**
 *  KEY ASSUMPTIONS
 * 
 * 1) All values in the cw (box size etc) are taken as fixed and are hard coded
 * 2) Only the colors specified in the cw are possible item colors
 * 3) Baskets and boxes will always be on the platform, never 'floating'
 * 4) baskets are not within each other
 * 5) If multiple baskets of the same colors exist, it doesn't matter which one the box goes into
 * 
 * 
 */

// Simple helper: create an end-effector pose at (x, y, z)
// with a basic top-down orientation for the Panda.
// Robot positioning

static geometry_msgs::msg::Pose make_pose(double x, double y, double z, const geometry_msgs::msg::Quaternion &o)
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


///////////////////////////////////////////////////////////////////////////////

cw1::cw1(const rclcpp::Node::SharedPtr &node)
{
  node_ = node;
  service_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  sensor_cb_group_  = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  g_cloud_ptr = std::make_shared<PointC>();
  g_cloud_filtered = std::make_shared<PointC>();
  g_cloud_plane = std::make_shared<PointC>();
  g_cloud_segmented_plane = std::make_shared<PointC>();
  g_cloud_cluster = std::make_shared<PointC>();
  g_tree_ptr = std::make_shared<pcl::search::KdTree<PointT>>();
  g_tree_ptr_euclidean = std::make_shared<pcl::search::KdTree<PointT>>();
  g_cloud_normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
  g_cloud_segmented_normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
  g_inliers_plane = std::make_shared<pcl::PointIndices>();
  g_coeff_plane = std::make_shared<pcl::ModelCoefficients>();

  // ── PCL filter parameters ──────────────────────────────────────────
  pcl_voxel_leaf_size_      = node_->declare_parameter("pcl.voxel_leaf_size",      pcl_voxel_leaf_size_);
  pcl_pass_min_             = node_->declare_parameter("pcl.pass_min",             pcl_pass_min_);
  pcl_pass_max_             = node_->declare_parameter("pcl.pass_max",             pcl_pass_max_);
  pcl_pass_axis_            = node_->declare_parameter("pcl.pass_axis",            pcl_pass_axis_);
  pcl_outlier_mean_k_       = node_->declare_parameter("pcl.outlier_mean_k",       pcl_outlier_mean_k_);
  pcl_outlier_stddev_       = node_->declare_parameter("pcl.outlier_stddev",       pcl_outlier_stddev_);
  pcl_normal_k_             = node_->declare_parameter("pcl.normal_k",             pcl_normal_k_);
  pcl_plane_normal_weight_  = node_->declare_parameter("pcl.plane_normal_weight",  pcl_plane_normal_weight_);
  pcl_plane_max_iterations_ = node_->declare_parameter("pcl.plane_max_iterations", pcl_plane_max_iterations_);
  pcl_plane_distance_       = node_->declare_parameter("pcl.plane_distance",       pcl_plane_distance_);
  pcl_cluster_tolerance_    = node_->declare_parameter("pcl.cluster_tolerance",    pcl_cluster_tolerance_);
  pcl_cluster_min_size_     = node_->declare_parameter("pcl.cluster_min_size",     pcl_cluster_min_size_);
  pcl_cluster_max_size_     = node_->declare_parameter("pcl.cluster_max_size",     pcl_cluster_max_size_);

  // ── Live-update callback ───────────────────────────────────────────
  param_cb_handle_= node_->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> &params)
    {

      RCLCPP_INFO_STREAM(node_->get_logger(), "Callback param reset triggered");

      for (const auto &p : params) {
        const auto &n = p.get_name();
        if      (n == "pcl.voxel_leaf_size")      pcl_voxel_leaf_size_      = p.as_double();
        else if (n == "pcl.pass_min")             pcl_pass_min_             = p.as_double();
        else if (n == "pcl.pass_max")             pcl_pass_max_             = p.as_double();
        else if (n == "pcl.pass_axis")            pcl_pass_axis_            = p.as_string();
        else if (n == "pcl.outlier_mean_k")       pcl_outlier_mean_k_       = static_cast<int>(p.as_int());
        else if (n == "pcl.outlier_stddev")       pcl_outlier_stddev_       = p.as_double();
        else if (n == "pcl.normal_k")             pcl_normal_k_             = static_cast<int>(p.as_int());
        else if (n == "pcl.plane_normal_weight")  pcl_plane_normal_weight_  = p.as_double();
        else if (n == "pcl.plane_max_iterations") pcl_plane_max_iterations_ = static_cast<int>(p.as_int());
        else if (n == "pcl.plane_distance")       pcl_plane_distance_       = p.as_double();
        else if (n == "pcl.cluster_tolerance")    pcl_cluster_tolerance_    = p.as_double();
        else if (n == "pcl.cluster_min_size")     pcl_cluster_min_size_     = static_cast<int>(p.as_int());
        else if (n == "pcl.cluster_max_size")     pcl_cluster_max_size_     = static_cast<int>(p.as_int());
      }
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;

      processCloud();
      return result;
    });



  g_cloud_ptr = std::make_shared<PointC>();
  g_cloud_filtered = std::make_shared<PointC>();
  g_cloud_plane = std::make_shared<PointC>();
  g_cloud_segmented_plane = std::make_shared<PointC>();
  g_cloud_cluster = std::make_shared<PointC>();
  g_tree_ptr = std::make_shared<pcl::search::KdTree<PointT>>();
  g_tree_ptr_euclidean = std::make_shared<pcl::search::KdTree<PointT>>();
  g_cloud_normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
  g_cloud_segmented_normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
  g_inliers_plane = std::make_shared<pcl::PointIndices>();
  g_coeff_plane = std::make_shared<pcl::ModelCoefficients>();

  // ── PCL filter parameters ──────────────────────────────────────────
  pcl_voxel_leaf_size_      = node_->declare_parameter("pcl.voxel_leaf_size",      pcl_voxel_leaf_size_);
  pcl_pass_min_             = node_->declare_parameter("pcl.pass_min",             pcl_pass_min_);
  pcl_pass_max_             = node_->declare_parameter("pcl.pass_max",             pcl_pass_max_);
  pcl_pass_axis_            = node_->declare_parameter("pcl.pass_axis",            pcl_pass_axis_);
  pcl_outlier_mean_k_       = node_->declare_parameter("pcl.outlier_mean_k",       pcl_outlier_mean_k_);
  pcl_outlier_stddev_       = node_->declare_parameter("pcl.outlier_stddev",       pcl_outlier_stddev_);
  pcl_normal_k_             = node_->declare_parameter("pcl.normal_k",             pcl_normal_k_);
  pcl_plane_normal_weight_  = node_->declare_parameter("pcl.plane_normal_weight",  pcl_plane_normal_weight_);
  pcl_plane_max_iterations_ = node_->declare_parameter("pcl.plane_max_iterations", pcl_plane_max_iterations_);
  pcl_plane_distance_       = node_->declare_parameter("pcl.plane_distance",       pcl_plane_distance_);
  pcl_cluster_tolerance_    = node_->declare_parameter("pcl.cluster_tolerance",    pcl_cluster_tolerance_);
  pcl_cluster_min_size_     = node_->declare_parameter("pcl.cluster_min_size",     pcl_cluster_min_size_);
  pcl_cluster_max_size_     = node_->declare_parameter("pcl.cluster_max_size",     pcl_cluster_max_size_);

  // ── Live-update callback ───────────────────────────────────────────
  param_cb_handle_= node_->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> &params)
    {

      RCLCPP_INFO_STREAM(node_->get_logger(), "Callback param reset triggered");

      for (const auto &p : params) {
        const auto &n = p.get_name();
        if      (n == "pcl.voxel_leaf_size")      pcl_voxel_leaf_size_      = p.as_double();
        else if (n == "pcl.pass_min")             pcl_pass_min_             = p.as_double();
        else if (n == "pcl.pass_max")             pcl_pass_max_             = p.as_double();
        else if (n == "pcl.pass_axis")            pcl_pass_axis_            = p.as_string();
        else if (n == "pcl.outlier_mean_k")       pcl_outlier_mean_k_       = static_cast<int>(p.as_int());
        else if (n == "pcl.outlier_stddev")       pcl_outlier_stddev_       = p.as_double();
        else if (n == "pcl.normal_k")             pcl_normal_k_             = static_cast<int>(p.as_int());
        else if (n == "pcl.plane_normal_weight")  pcl_plane_normal_weight_  = p.as_double();
        else if (n == "pcl.plane_max_iterations") pcl_plane_max_iterations_ = static_cast<int>(p.as_int());
        else if (n == "pcl.plane_distance")       pcl_plane_distance_       = p.as_double();
        else if (n == "pcl.cluster_tolerance")    pcl_cluster_tolerance_    = p.as_double();
        else if (n == "pcl.cluster_min_size")     pcl_cluster_min_size_     = static_cast<int>(p.as_int());
        else if (n == "pcl.cluster_max_size")     pcl_cluster_max_size_     = static_cast<int>(p.as_int());
      }
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;

      processCloud();
      return result;
    });



  // advertise solutions for coursework tasks
  t1_service_ = node_->create_service<cw1_world_spawner::srv::Task1Service>(
    "/task1_start", std::bind(&cw1::t1_callback, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, service_cb_group_);
  t2_service_ = node_->create_service<cw1_world_spawner::srv::Task2Service>(
    "/task2_start", std::bind(&cw1::t2_callback, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, service_cb_group_);
  t3_service_ = node_->create_service<cw1_world_spawner::srv::Task3Service>(
    "/task3_start", std::bind(&cw1::t3_callback, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, service_cb_group_);

  g_pub_cloud = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/cw1_cloud/cloud", 1);
  g_pub_passthrough = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/cw1_cloud/cloud_passthrough", 1);
  g_pub_outlier = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/cw1_cloud/cloud_outlier", 1);
  g_pub_plane = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/cw1_cloud/cloud_plane", 1);
  g_pub_cluster1 = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/cw1_cloud/cloud_cluster1", 1);
  g_pub_cluster2 = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/cw1_cloud/cloud_cluster2", 1);
  g_pub_cluster3 = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/cw1_cloud/cloud_cluster3", 1);
  g_pub_cluster4 = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/cw1_cloud/cloud_cluster4", 1);
  g_pub_cluster5 = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/cw1_cloud/cloud_cluster5", 1);
  g_pub_cluster6 = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/cw1_cloud/cloud_cluster6", 1);

  g_pub_clusters = {g_pub_cluster1, g_pub_cluster2, g_pub_cluster3, g_pub_cluster4, g_pub_cluster5, g_pub_cluster6};

  


  // Service and sensor callbacks use separate callback groups to align with the
  // current runtime architecture used in cw1_team_0.
  rclcpp::SubscriptionOptions joint_state_sub_options;
  joint_state_sub_options.callback_group = sensor_cb_group_;
  auto joint_state_qos = rclcpp::QoS(rclcpp::KeepLast(50));
  joint_state_qos.reliable();
  joint_state_qos.durability_volatile();
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::QoS(50).reliable(),
    [this](const sensor_msgs::msg::JointState::ConstSharedPtr) {
      joint_state_msg_count_.fetch_add(1, std::memory_order_relaxed); }, jo);

  rclcpp::SubscriptionOptions cloud_sub_options;
  cloud_sub_options.callback_group = sensor_cb_group_;
  auto cloud_qos = rclcpp::QoS(rclcpp::KeepLast(10));
  cloud_qos.reliable();
  cloud_qos.durability_volatile();
  cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/r200/camera/depth_registered/points", cloud_qos,
    [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
      latest_cloud_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>(*msg);
      const int64_t stamp_ns =
        static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL +
        static_cast<int64_t>(msg->header.stamp.nanosec);
      latest_cloud_stamp_ns_.store(stamp_ns, std::memory_order_relaxed);
      cloud_msg_count_.fetch_add(1, std::memory_order_relaxed);
    },
    cloud_sub_options);

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

  (void)request;
  (void)response;
  RCLCPP_INFO_STREAM(
    node_->get_logger(),
    "Task 2 callback triggered (template stub). joint_msgs=" <<
      joint_state_msg_count_.load(std::memory_order_relaxed) <<
      ", cloud_msgs=" << cloud_msg_count_.load(std::memory_order_relaxed));

  static const std::string planning_group = "panda_arm";
  moveit::planning_interface::MoveGroupInterface move_group2(node_, planning_group);


  move_group2.setPlanningTime(5.0);
  move_group2.setMaxVelocityScalingFactor(0.2);
  move_group2.setMaxAccelerationScalingFactor(0.2);
  if (!moveToBirdeye(move_group2))
  {
    //failed to get to birdseye postion - manually defined position by joint angles
    return;
  }

  // move_group2.setPlanningTime(5.0);
  // move_group2.setMaxVelocityScalingFactor(0.2);
  // move_group2.setMaxAccelerationScalingFactor(0.2);
  // // First movement test: go to a hover pose above the cube
  // geometry_msgs::msg::Pose current_pose = move_group2.getCurrentPose().pose;

  geometry_msgs::msg::Pose target_pose = current_pose;
  target_pose.position.z += 0.31; // Raise by 31cm

  // move_group2.setPoseTarget(target_pose);

  // moveit::planning_interface::MoveGroupInterface::Plan plan2;
  // bool success = (
  // move_group2.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);
  // if (!success) {
  //   RCLCPP_ERROR(node_->get_logger(), "Planning to hover pose failed");
  //   // return;
  // }

  auto exec_result2 = move_group2.execute(plan2);
  if (exec_result2 != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Execution to hover pose failed");
    // return;
  }

  //5 second wait as pose moves

  std::this_thread::sleep_for(std::chrono::milliseconds(5000));

  processCloud();

  // if (latest_cloud_msg_) {

  //     std_msgs::msg::Header header;
  //     header.frame_id = "color";
  //     header.stamp = latest_cloud_msg_->header.stamp;

  //     RCLCPP_INFO_STREAM(node_->get_logger(), "creating cloud");
  //     rosTopicToCloud(latest_cloud_msg_);
  //     pubFilteredPCMsg(g_pub_cloud, *g_cloud_filtered, header);
  //     applyPassthrough(pcl_pass_min_, pcl_pass_max_, pcl_pass_axis_);
  //     pubFilteredPCMsg(g_pub_passthrough, *g_cloud_filtered, header);
  //     applyOutlierRemoval(pcl_outlier_mean_k_, pcl_outlier_stddev_);
  //     pubFilteredPCMsg(g_pub_outlier, *g_cloud_filtered, header);
  //     findNormals(pcl_normal_k_);
  //     segmentPlane(pcl_plane_normal_weight_, pcl_plane_max_iterations_, pcl_plane_distance_);
  //     pubFilteredPCMsg(g_pub_plane, *g_cloud_segmented_plane, header);
  //     extractEuclideanClusters(pcl_cluster_tolerance_, pcl_cluster_min_size_, pcl_cluster_max_size_);
      



      



  // } else {
  //     RCLCPP_WARN(node_->get_logger(), "No point cloud message received yet.");
  // }  



}

///////////////////////////////////////////////////////////////////////////////

void
cw1::t3_callback(
  const std::shared_ptr<cw1_world_spawner::srv::Task3Service::Request> request,
  std::shared_ptr<cw1_world_spawner::srv::Task3Service::Response> response)
{
  /* function which should solve task 3 */

  (void)request;
  (void)response;
  RCLCPP_INFO_STREAM(
    node_->get_logger(),
    "Task 3 callback triggered (template stub). joint_msgs=" <<
      joint_state_msg_count_.load(std::memory_order_relaxed) <<
      ", cloud_msgs=" << cloud_msg_count_.load(std::memory_order_relaxed));

  static const std::string planning_group = "panda_arm";
  moveit::planning_interface::MoveGroupInterface move_group2(node_, planning_group);


  move_group2.setPlanningTime(5.0);
  move_group2.setMaxVelocityScalingFactor(0.2);
  move_group2.setMaxAccelerationScalingFactor(0.2);
  if (!moveToBirdeye(move_group2))
  {
    //failed to get to birdseye postion - manually defined position by joint angles
    return;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  //sement out plane
  segmentPlane();

  //get respective point clouds
  std::vector<PointCPtr> all_clouds = getBasketClouds();
  
  std::vector<PointCPtr> baskets;
  std::vector<PointCPtr> boxes;

  for (size_t i = 0; i < all_clouds.size(); i++)
  {
    PointT min, max;

    pcl::getMinMax3D(*all_clouds[i], min, max);

    float dx = max.x - min.x;
    float dy = max.y - min.y;

    RCLCPP_INFO(node_->get_logger(), "Cloud %zu has dimensions dx=%.3f dy=%.3f", i, dx, dy);

    if (dx < 0.07 && dy < 0.07)
    {
      //its a box!
      boxes.push_back(all_clouds[i]);
    }
    else
    {
      //its a basket!
      baskets.push_back(all_clouds[i]);

      if (i < 6){
        std_msgs::msg::Header header;
        header.frame_id = "color";
        header.stamp = latest_cloud_msg_->header.stamp;
        pubFilteredPCMsg(g_pub_clusters[i], *all_clouds[i], header);
      }
    }
  }



  RCLCPP_INFO(node_->get_logger(), "T3: got both clouds"); 
  RCLCPP_INFO(node_->get_logger(), "T3: Num Baskets: %zu, num boxes: %zu", baskets.size(), boxes.size());    
   

  //get colors/coords
  std::vector<Eigen::Vector3f> box_coords;
  std::vector<std::string> box_colors;

  std::vector<Eigen::Vector3f> basket_coords;
  std::vector<std::string> basket_colors;

  //store baskets first. will compare them with each box
  for (size_t i = 0; i < baskets.size(); i++)
  {
    basket_coords.push_back(toWorldFrame(getCentroid(*baskets[i])));
    basket_colors.push_back(colorOfPointCloud(*baskets[i], 0.2));

    RCLCPP_INFO(node_->get_logger(), "Basket %zu is %s: x=%.3f y=%.3f z=%.3f", i, basket_colors[i].c_str(), basket_coords[i].x(), basket_coords[i].y(), basket_coords[i].z());
  }

  RCLCPP_INFO(node_->get_logger(), "T3: Finished storing basket details");    


  struct CloudPair {
  Eigen::Vector3f cube;
  Eigen::Vector3f basket;
  };

  std::vector<CloudPair> pairs;


  //pair each box with known baskets
  for (size_t i = 0; i < boxes.size(); i++)
  {

    Eigen::Vector3f c = toWorldFrame(getCentroid(*boxes[i]));
    std::string color = colorOfPointCloud(*boxes[i], 0.2);

    //housekeeping
    box_coords.push_back(c);
    box_colors.push_back(color);

    RCLCPP_INFO(node_->get_logger(), "Box %zu is %s: x=%.3f y=%.3f z=%.3f", i, color.c_str(), c.x(), c.y(), c.z());
    
    for (size_t j = 0; j < baskets.size(); j++)
    {

      if (color == basket_colors[j])
      {
        //its a match!!

        RCLCPP_INFO(node_->get_logger(), "T3: Matched Basket %zu with %zu of color %s", i, j, color.c_str());    


        pairs.push_back({c, basket_coords[j]});
        break;
      }

    }
  
  }

  RCLCPP_INFO(node_->get_logger(), "T3: Have %zu pairs", pairs.size());    



  std::this_thread::sleep_for(std::chrono::milliseconds(1000));


  for (size_t i = 0; i < pairs.size(); i++)
  {
    const auto& pair = pairs[i];
    //move it command here

    RCLCPP_INFO(node_->get_logger(), "Move (x=%.3f y=%.3f z=%.3f) to(x=%.3f y=%.3f z=%.3f) ", pair.cube.x(), pair.cube.y(), pair.cube.z(), pair.basket.x(), pair.basket.y(), pair.basket.z());

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  }

}


void cw1::rosTopicToCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_input_msg)
{
  g_input_pc_frame_id = cloud_input_msg->header.frame_id;

  pcl::fromROSMsg(*cloud_input_msg, *g_cloud_ptr);

  *g_cloud_filtered = *g_cloud_ptr;

  RCLCPP_INFO_STREAM(node_->get_logger(), "saved cloud");
}

void cw1::applyVoxelGrid(double g_leaf_size)
{
  
  PointCPtr output_cloud(new PointC);

  g_vx.setInputCloud(g_cloud_filtered);
  g_vx.setLeafSize(g_leaf_size, g_leaf_size, g_leaf_size);
  g_vx.filter(*output_cloud);

  g_cloud_filtered.swap(output_cloud);

  RCLCPP_INFO_STREAM(node_->get_logger(), "grid cloud");


}

void cw1::applyPassthrough(double g_pass_min, double g_pass_max, std::string g_pass_axis)
{
  
  PointCPtr output_cloud(new PointC);

  g_pt.setInputCloud(g_cloud_filtered);
  g_pt.setFilterFieldName(g_pass_axis);
  g_pt.setFilterLimits(g_pass_min, g_pass_max);
  g_pt.filter(*output_cloud);
  g_cloud_filtered.swap(output_cloud);


}

void cw1::applyOutlierRemoval(int g_outlier_mean_k, double g_outlier_stddev)
{
  
  PointCPtr output_cloud(new PointC);

  g_sor.setInputCloud(g_cloud_filtered);
  g_sor.setMeanK(g_outlier_mean_k);
  g_sor.setStddevMulThresh(g_outlier_stddev);
  g_sor.filter(*output_cloud);
  g_cloud_filtered.swap(output_cloud);

}

void cw1::findNormals(int g_normal_k)
{
  g_ne.setInputCloud(g_cloud_filtered);
  g_ne.setSearchMethod(g_tree_ptr);
  g_ne.setKSearch(g_normal_k);
  g_ne.compute(*g_cloud_normals);
}

void cw1::segmentPlane(double g_plane_normal_dist_weight, int g_plane_max_iterations, double g_plane_distance)
{
  // TODO(student-9): Implement normal-plane segmentation.

  //Configure model
  g_seg.setOptimizeCoefficients(true);
  g_seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
  g_seg.setNormalDistanceWeight(g_plane_normal_dist_weight);
  g_seg.setMethodType(pcl::SAC_RANSAC);
  g_seg.setMaxIterations(g_plane_max_iterations);
  g_seg.setDistanceThreshold(g_plane_distance);

  //Set cloud to segment
  g_seg.setInputCloud(g_cloud_filtered);
  g_seg.setInputNormals(g_cloud_normals);

  //segment
  g_seg.segment(*g_inliers_plane, *g_coeff_plane);

  //extract point cloud that is the plane with inliers
  g_extract_pc.setInputCloud(g_cloud_filtered);
  g_extract_pc.setIndices(g_inliers_plane);
  g_extract_pc.setNegative(false);
  g_extract_pc.filter(*g_cloud_plane);

  //extract point cloud that is NOT the plane (outliers). Store in filtered 2
  g_extract_pc.setNegative(true);
  g_extract_pc.filter(*g_cloud_segmented_plane);

  //remove normals from the normal cloud that are in the plane
  //normals2 is just normals of non-plane items
  g_extract_normals.setNegative(true);
  g_extract_normals.setInputCloud(g_cloud_normals);
  g_extract_normals.setIndices(g_inliers_plane);
  g_extract_normals.filter(*g_cloud_segmented_normals);
}


std::vector<pcl::PointIndices> cw1::extractEuclideanClusters(double clusterTolerance, int minClusterSize, int maxClusterSize)
{
  g_tree_ptr_euclidean->setInputCloud(g_cloud_segmented_plane);
  //Configure clustering
  std::vector<pcl::PointIndices> cluster_indices;
  g_extract_euclidean.setClusterTolerance(clusterTolerance); // 2cm
  g_extract_euclidean.setMinClusterSize(minClusterSize);
  g_extract_euclidean.setMaxClusterSize(maxClusterSize);
  g_extract_euclidean.setSearchMethod(g_tree_ptr_euclidean);
  g_extract_euclidean.setInputCloud(g_cloud_segmented_plane);

  //Extract clusters
  g_extract_euclidean.extract(cluster_indices);
  
  int num_cluster = 0;

  return cluster_indices;
  std::size_t largest_size = 0;

  int num_cluster = 0;

  for (const auto& cluster : cluster_indices)
  {


    
    PointCPtr cloud_cluster(new PointC);
    for (const auto& idx : cluster.indices) {
      cloud_cluster->push_back((*g_cloud_segmented_plane)[idx]);
    }
    cloud_cluster->width = cloud_cluster->size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std_msgs::msg::Header header;
    header.frame_id = "color";
    header.stamp = latest_cloud_msg_->header.stamp;


    pubFilteredPCMsg(g_pub_clusters[num_cluster], *cloud_cluster, header);

    if (cloud_cluster->size() == 0)
    {
      continue;
    }

    if (cloud_cluster->size() > largest_size)
    {
      largest_size = cloud_cluster->size();
      g_cloud_cluster = cloud_cluster;
    }

    

    Eigen::Vector3f c = getCentroid(*cloud_cluster);

    std::string c_name = colorOfPointCloud(*cloud_cluster, 0.2);

    RCLCPP_INFO(node_->get_logger(), "Centroid %d is %s: x=%.3f y=%.3f z=%.3f", num_cluster, c_name.c_str(), c.x(), c.y(), c.z());
    
    num_cluster = num_cluster + 1;

    if (num_cluster >= 6)
    {
      break;
    }

  }
}

Eigen::Vector3f cw1::getCentroid(PointC &in_cloud_ptr)
{
  
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(in_cloud_ptr, centroid);

  return centroid.head<3>();  // drops the homogeneous w component
}

void cw1::pubFilteredPCMsg(
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pc_pub,
    PointC &pc,
    const std_msgs::msg::Header &header)
{

  // publish type
  sensor_msgs::msg::PointCloud2 output;

  RCLCPP_INFO(node_->get_logger(), "FRAME ID: %s", g_input_pc_frame_id.c_str());

  //pass input cloud, output PointCloud2 by reference
  pcl::toROSMsg(pc, output);
  output.header = header;
  pc_pub->publish(output);

}

void cw1::processCloud()
{
  if (!latest_cloud_msg_) {
    RCLCPP_WARN(node_->get_logger(), "reprocessCloud: no cloud yet, skipping.");
    return;
  }
  std_msgs::msg::Header header;
  header.frame_id = "color";
  header.stamp = latest_cloud_msg_->header.stamp;

  RCLCPP_INFO_STREAM(node_->get_logger(), "creating cloud");
  rosTopicToCloud(latest_cloud_msg_);
  pubFilteredPCMsg(g_pub_cloud, *g_cloud_filtered, header);
  applyPassthrough(pcl_pass_min_, pcl_pass_max_, pcl_pass_axis_);
  pubFilteredPCMsg(g_pub_passthrough, *g_cloud_filtered, header);
  applyOutlierRemoval(pcl_outlier_mean_k_, pcl_outlier_stddev_);
  pubFilteredPCMsg(g_pub_outlier, *g_cloud_filtered, header);
  findNormals(pcl_normal_k_);
  segmentPlane(pcl_plane_normal_weight_, pcl_plane_max_iterations_, pcl_plane_distance_);
  pubFilteredPCMsg(g_pub_plane, *g_cloud_segmented_plane, header);
  extractEuclideanClusters(pcl_cluster_tolerance_, pcl_cluster_min_size_, pcl_cluster_max_size_);

}

std::string cw1::colorOfPointCloud(PointC &in_cloud_ptr, float threshold)
{

  float r = 0;
  float g = 0;
  float b = 0;

  //average all point colors in the cloud

  for (const auto & pt : in_cloud_ptr.points)
  {
    r = r + (pt.r / 255.0);
    g = g + (pt.g / 255.0);
    b = b + (pt.b / 255.0);
  }


  r = r / in_cloud_ptr.size();
  g = g / in_cloud_ptr.size();
  b = b / in_cloud_ptr.size();


  float min_dist = 1000;
  int min_color_idx;

  RCLCPP_INFO(node_->get_logger(), "num_pts = %d, r=%.3f g=%.3f b=%.3f", static_cast<int>(in_cloud_ptr.size()), r, g, b);


  // find closest color
  for (size_t i = 0; i < num_colors; i++)
  {

    std::array<float, 3> color = colors[i];
    //distance between average and saved color
    float dist = std::sqrt(std::pow(r - color[0], 2) + std::pow(g - color[1], 2) + std::pow(b - color[2], 2));
    RCLCPP_INFO(node_->get_logger(), "dist=%.3f", dist);


    if (dist < min_dist)
    {
      min_dist = dist;
      min_color_idx = i;
    }

  }


  //check to see if closest color is in range
  if (min_dist < threshold)
  {
    return color_names[min_color_idx];
  }
  else
  {
    return no_color;
  }

}

  




