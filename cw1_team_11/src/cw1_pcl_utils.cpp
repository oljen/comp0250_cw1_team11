/*
 * Student starter node for COMP0250 ROS 2 PCL tutorial.
 * This package compiles, subscribes, and publishes nothing by default.
 * Fill in TODO blocks as part of the lab.
 */

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>

//IMPORTANT TYPEDEF

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;

class PCL_CW1 final : public rclcpp::Node
{
public:
  PCL_CW1()

  //Predefined global pointers for us
    g_cloud_ptr(new PointC),
    g_cloud_filtered(new PointC),
    g_cloud_plane(new PointC),
    g_tree_ptr(new pcl::search::KdTree<PointT>()),
    g_cloud_normals(new pcl::PointCloud<pcl::Normal>),
    g_cloud_segmented_normals(new pcl::PointCloud<pcl::Normal>),
    g_inliers_plane(new pcl::PointIndices),
    g_coeff_plane(new pcl::ModelCoefficients),
    g_cloud_segmented_plane(new PointC)
  {
    // g_input_topic = this->declare_parameter<std::string>(
    //   "input_topic", "/r200/camera/depth_registered/points");

    // g_output_cloud_topic = this->declare_parameter<std::string>(
    //   "output_cloud_topic", "/pcl_tutorial/filtered");
    // g_output_cyl_pt_topic = this->declare_parameter<std::string>(
    //   "output_cyl_pt_topic", "/pcl_tutorial/centroid");

    // g_enable_voxel = this->declare_parameter<bool>("enable_voxel", false);
    // g_leaf_size = this->declare_parameter<double>("leaf_size", 0.01);

    // g_enable_pass = this->declare_parameter<bool>("enable_pass", false);
    // g_pass_axis = this->declare_parameter<std::string>("pass_axis", "x");
    // g_pass_min = this->declare_parameter<double>("pass_min", 0.0);
    // g_pass_max = this->declare_parameter<double>("pass_max", 0.7);

    // g_enable_outlier = this->declare_parameter<bool>("enable_outlier", false);
    // g_outlier_mean_k = this->declare_parameter<int>("outlier_mean_k", 20);
    // g_outlier_stddev = this->declare_parameter<double>("outlier_stddev", 1.0);

    // g_do_plane = this->declare_parameter<bool>("do_plane", false);
    // g_do_cylinder = this->declare_parameter<bool>("do_cylinder", false);

    // g_normal_k = this->declare_parameter<int>("normal_k", 50);
    // g_plane_normal_dist_weight = this->declare_parameter<double>("plane_normal_dist_weight", 0.1);
    // g_plane_max_iterations = this->declare_parameter<int>("plane_max_iterations", 100);
    // g_plane_distance = this->declare_parameter<double>("plane_distance", 0.03);

    // g_cylinder_normal_dist_weight = this->declare_parameter<double>("cylinder_normal_dist_weight", 0.1);
    // g_cylinder_max_iterations = this->declare_parameter<int>("cylinder_max_iterations", 10000);
    // g_cylinder_distance = this->declare_parameter<double>("cylinder_distance", 0.05);
    // g_cylinder_radius_min = this->declare_parameter<double>("cylinder_radius_min", 0.0);
    // g_cylinder_radius_max = this->declare_parameter<double>("cylinder_radius_max", 0.1);

    // g_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    //   g_input_topic, rclcpp::SensorDataQoS(),
    //   std::bind(&PCL_CW1::cloudCallBackOne, this, std::placeholders::_1));

    // g_pub_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>(g_output_cloud_topic, 1);
    // g_pub_cyl_pt = this->create_publisher<geometry_msgs::msg::PointStamped>(g_output_cyl_pt_topic, 1);
  }

private:
  void rosTopicToCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_input_msg)
  {
    g_input_pc_frame_id = cloud_input_msg->header.frame_id;

    pcl::fromROSMsg(*cloud_input_msg, *g_cloud_ptr);

    *g_cloud_filtered = *g_cloud_ptr;
 
  }


  void applyVoxelGrid(double g_leaf_size)
  {
    
    PointCPtr output_cloud(new PointC);

    g_vx.setInputCloud(g_cloud_filtered);
    g_vx.setLeafSize(g_leaf_size, g_leaf_size, g_leaf_size);
    g_vx.filter(*output_cloud);

    g_cloud_filtered.swap(output_cloud);

  }

  void applyPassthrough(double g_pass_min, double g_pass_max, std::string g_pass_axis)
  {
    
    PointCPtr output_cloud(new PointC);

    g_pt.setInputCloud(g_cloud_filtered);
    g_pt.setFilterFieldName(g_pass_axis);
    g_pt.setFilterLimits(g_pass_min, g_pass_max);
    g_pt.filter(*output_cloud);
    g_cloud_filtered.swap(output_cloud);
  }

  void applyOutlierRemoval(int g_outlier_mean_k, double g_outlier_stddev)
  {
    
    PointCPtr output_cloud(new PointC);

    g_sor.setInputCloud(g_cloud_filtered);
    g_sor.setMeanK(g_outlier_mean_k);
    g_sor.setStddevMulThresh(g_outlier_stddev);
    g_sor.filter(*output_cloud);
    g_cloud_filtered.swap(output_cloud);

  }

  void findNormals(int g_normal_k)
  {
    g_ne.setInputCloud(g_cloud_filtered);
    g_ne.setSearchMethod(g_tree_ptr);
    g_ne.setKSearch(g_normal_k);
    g_ne.compute(*g_cloud_normals);
  }

  void segmentPlane(double g_plane_normal_dist_weight, int g_plane_max_iterations, double g_plane_distance, PointCPtr &in_cloud_ptr)
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

  


  void findCylPose(PointCPtr &in_cloud_ptr)
  {
    // TODO(student-11): Compute and publish cylinder centroid in target frame.
    
    if (in_cloud_ptr->empty())
    {
      return;
    }

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*in_cloud_ptr, centroid);

    geometry_msgs::msg::PointStamped cyl_pt_msg;
    cyl_pt_msg.header.frame_id = g_input_pc_frame_id;
    cyl_pt_msg.header.stamp = this->now();
    cyl_pt_msg.point.x = centroid[0];
    cyl_pt_msg.point.y = centroid[1];
    cyl_pt_msg.point.z = centroid[2];

    geometry_msgs::msg::PointStamped cyl_pt_msg_out;

    publishPose(cyl_pt_msg);

    try {
      cyl_pt_msg_out = g_tf_buffer.transform(cyl_pt_msg, "panda_link0", tf2::durationFromSec(0.0));

      publishPose(cyl_pt_msg_out);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(get_logger(), "TF transform failed: %s", ex.what());
    }
  
  }


private:
  // std::string g_input_topic;
  // std::string g_output_cloud_topic;
  // std::string g_output_cyl_pt_topic;

  // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr g_sub;
  // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr g_pub_cloud;
  // rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr g_pub_cyl_pt;

  tf2_ros::Buffer g_tf_buffer{this->get_clock()};
  tf2_ros::TransformListener g_tf_listener{g_tf_buffer};

  std::string g_input_pc_frame_id;

  PointCPtr g_cloud_ptr;
  PointCPtr g_cloud_filtered;
  PointCPtr g_cloud_plane;
  PointCPtr g_cloud_segmented_plane;
  PointCPtr g_cloud_cylinder;

  pcl::VoxelGrid<PointT> g_vx;
  pcl::PassThrough<PointT> g_pt;
  pcl::StatisticalOutlierRemoval<PointT> g_sor;

  pcl::search::KdTree<PointT>::Ptr g_tree_ptr;
  pcl::NormalEstimation<PointT, pcl::Normal> g_ne;
  pcl::PointCloud<pcl::Normal>::Ptr g_cloud_normals;
  pcl::PointCloud<pcl::Normal>::Ptr g_cloud_segmented_normals;

  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> g_seg;
  pcl::ExtractIndices<PointT> g_extract_pc;
  pcl::ExtractIndices<pcl::Normal> g_extract_normals;

  pcl::PointIndices::Ptr g_inliers_plane;
  pcl::ModelCoefficients::Ptr g_coeff_plane;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PCL_CW1>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
