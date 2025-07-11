#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>


namespace PCL_Clustering {



class Pcl_Clustering : public rclcpp::Node {

 public:

  Pcl_Clustering();
  void setup();
  void clustering();
  void pub_cluster();
 private:

  void PCL_Callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

  std::string input_topic;
  double cluster_tolerance;
  int min_cluster_size;
  int max_cluster_size;
  double voxel_resolution;
  double ground_thresh;
  double sky_thresh;
  double coloring_distance;
  std::string frame_id;

  pcl::PointCloud<pcl::PointXYZ>::Ptr filt_voxel;
  std::vector<pcl::PointXYZ> cluster_centroids;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cluster;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_pub;
};


}
