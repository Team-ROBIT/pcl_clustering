#pragma once

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <cmath>
#include <vector>

#include "base_msgs/msg/mission_status.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

namespace pcl_clustering {

class Pcl_Clustering : public rclcpp::Node {
 public:
  Pcl_Clustering();
  void setup();
  void clustering();
  //void pub_cluster();
  void match_point();

 private:
  void PCL_Callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
  void master_cb(const base_msgs::msg::MissionStatus::ConstSharedPtr& msg);
  void vision_cb(const std_msgs::msg::Float32MultiArray::ConstSharedPtr& msg);
  void pub_master();

  // cluster
  std::string input_topic;
  double cluster_tolerance;
  int min_cluster_size;
  int max_cluster_size;
  double voxel_resolution;
  double ground_thresh;
  double sky_thresh;
  double coloring_distance;
  std::string frame_id;

  // defense
  std::vector<double> cam_intrinsic;
  double theta_h = 0, theta_v = 0;
  base_msgs::msg::MissionStatus master_msg;
  std_msgs::msg::Float32MultiArray vision_msg;
  bool check_none=false;

  //
  pcl::PointCloud<pcl::PointXYZ>::Ptr filt_voxel;
  //std::vector<pcl::PointXYZ> cluster_centroids;
  std::vector<std::tuple<pcl::PointXYZ, float, float, float>> cluster_centroids; //centroid, angle_h, angle_v, dist
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cluster;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_pub;

  rclcpp::Subscription<base_msgs::msg::MissionStatus>::SharedPtr master_sub;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr vision_sub;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr master_pub;
};

}  // namespace pcl_clustering
