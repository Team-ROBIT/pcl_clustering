#include <functional>
#include <pcl_clustering/pcl_clustering.hpp>

namespace pcl_clustering {

Pcl_Clustering::Pcl_Clustering() : Node("pcl_clustering") {
  this->declare_parameter<std::string>("input_topic", "/pcl_data");
  this->declare_parameter<double>("cluster_tolerance", 0.0);
  this->declare_parameter<int>("min_cluster_size", 0);
  this->declare_parameter<int>("max_cluster_size", 0);
  this->declare_parameter<double>("coloring_distance", 0.0);
  this->declare_parameter<double>("voxel_resolution", 0.0);
  this->declare_parameter<double>("ground_thresh", 0.0);
  this->declare_parameter<double>("sky_thresh", 0.0);
  this->declare_parameter<std::string>("frame_id", "velodyne");
  setup();
}

void Pcl_Clustering::setup() {
  // parameters
  this->get_parameter("input_topic", input_topic);
  this->get_parameter("cluster_tolerance", cluster_tolerance);  
  RCLCPP_INFO(this->get_logger(), "coloring_distance: %.2f", coloring_distance);
  this->get_parameter("coloring_distance", coloring_distance);
  RCLCPP_INFO(this->get_logger(), "coloring_distance: %.2f", coloring_distance);
  this->get_parameter("voxel_resolution", voxel_resolution);
  this->get_parameter("ground_thresh", ground_thresh);
  this->get_parameter("sky_thresh", sky_thresh);
  this->get_parameter("min_cluster_size", min_cluster_size);
  this->get_parameter("max_cluster_size", max_cluster_size);
  this->get_parameter("frame_id", frame_id);
  RCLCPP_INFO(this->get_logger(), "input_topic: %s", input_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "cluster_tolerance: %.2f", cluster_tolerance);
  RCLCPP_INFO(this->get_logger(), "coloring_distance: %.2f", coloring_distance);
  RCLCPP_INFO(this->get_logger(), "voxel_resolution: %.2f", voxel_resolution);
  RCLCPP_INFO(this->get_logger(), "ground_thresh: %.2f", ground_thresh);
  RCLCPP_INFO(this->get_logger(), "sky_thresh: %.2f", sky_thresh);
  RCLCPP_INFO(this->get_logger(), "min_cluster_size: %d", min_cluster_size);
  RCLCPP_INFO(this->get_logger(), "max_cluster_size: %d", max_cluster_size);
  RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id.c_str());

  // sub pub
  pcl_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, 10,
      std::bind(&Pcl_Clustering::PCL_Callback, this, std::placeholders::_1));
  cluster_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/cluster_points", 10);

  // initial setting
  filt_voxel = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  colored_cluster = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
}

void Pcl_Clustering::PCL_Callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *cloud);

  // voxel
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_map(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(cloud);
  voxel.setLeafSize(voxel_resolution, voxel_resolution, voxel_resolution);
  voxel.filter(*voxel_map);

  filt_voxel->clear();
  for (const auto& pt : voxel_map->points) {
    if (pt.z > ground_thresh && pt.z < sky_thresh) {
      filt_voxel->points.push_back(pt);
    }
  }
  filt_voxel->width = filt_voxel->points.size();
  filt_voxel->height = 1;
  filt_voxel->is_dense = true;
}

void Pcl_Clustering::clustering() {
  // clustering with kdtree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(
      new pcl::search::KdTree<pcl::PointXYZ>());
  kdtree->setInputCloud(filt_voxel);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(cluster_tolerance);
  ec.setMinClusterSize(min_cluster_size);
  ec.setMaxClusterSize(max_cluster_size);
  ec.setSearchMethod(kdtree);
  ec.setInputCloud(filt_voxel);
  ec.extract(cluster_indices);

  // calc centroid , and coloring objs under distance
  cluster_centroids.clear();
  colored_cluster->clear();

  for (const auto& indices : cluster_indices) {
    // calc centroid
    float x_sum = 0, y_sum = 0, z_sum = 0;
    size_t n = indices.indices.size();
    for (int idx : indices.indices) {
      const auto& pt = filt_voxel->points[idx];
      x_sum += pt.x;
      y_sum += pt.y;
      z_sum += pt.z;
    }
    pcl::PointXYZ centroid{x_sum / n, y_sum / n, z_sum / n};
    cluster_centroids.push_back(centroid);

    double dist = std::sqrt(centroid.x * centroid.x + centroid.y * centroid.y +
                            centroid.z * centroid.z);
    uint8_t r, g, b;
    if (dist < coloring_distance) {
      r = 255;
      g = 0;
      b = 0;
    } else {
      r = 0;
      g = 0;
      b = 255;
    }
    for (int idx : indices.indices) {
      const auto& pt = filt_voxel->points[idx];
      pcl::PointXYZRGB pt_rgb;
      pt_rgb.x = pt.x;
      pt_rgb.y = pt.y;
      pt_rgb.z = pt.z;
      pt_rgb.r = r;
      pt_rgb.g = g;
      pt_rgb.b = b;
      colored_cluster->points.push_back(pt_rgb);
    }
  }
}

void Pcl_Clustering::pub_cluster() {
  sensor_msgs::msg::PointCloud2 cluster_msg;
  pcl::toROSMsg(*colored_cluster, cluster_msg);
  cluster_msg.header.frame_id = frame_id;
  cluster_msg.header.stamp = this->get_clock()->now();
  cluster_pub->publish(cluster_msg);
}

}  // namespace pcl_clustering

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::Rate rate(30);
  auto clustering_node = std::make_shared<pcl_clustering::Pcl_Clustering>();
  RCLCPP_INFO(clustering_node->get_logger(), "start");
  //rclcpp::spin_some(clustering_node);
  // RCLCPP_INFO(clustering_node->get_logger(), "setup");
  // clustering_node->setup();

  while (rclcpp::ok()) {
    rclcpp::spin_some(clustering_node);
    clustering_node->clustering();
    clustering_node->pub_cluster();
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
