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
  this->declare_parameter<std::vector<double>>("cam_intrinsic",
                                               {1.0, 2.0, 3.0, 4.0});
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
  this->get_parameter("cam_intrinsic", cam_intrinsic);

  RCLCPP_INFO(this->get_logger(), "input_topic: %s", input_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "cluster_tolerance: %.2f", cluster_tolerance);
  RCLCPP_INFO(this->get_logger(), "coloring_distance: %.2f", coloring_distance);
  RCLCPP_INFO(this->get_logger(), "voxel_resolution: %.2f", voxel_resolution);
  RCLCPP_INFO(this->get_logger(), "ground_thresh: %.2f", ground_thresh);
  RCLCPP_INFO(this->get_logger(), "sky_thresh: %.2f", sky_thresh);
  RCLCPP_INFO(this->get_logger(), "min_cluster_size: %d", min_cluster_size);
  RCLCPP_INFO(this->get_logger(), "max_cluster_size: %d", max_cluster_size);
  RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id.c_str());
  RCLCPP_INFO(this->get_logger(), "cam_intrinsic: [%f, %f, %f, %f]",
              cam_intrinsic[0], cam_intrinsic[1], cam_intrinsic[2],
              cam_intrinsic[3]);

  // sub
  pcl_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, 10,
      std::bind(&Pcl_Clustering::PCL_Callback, this, std::placeholders::_1));
  master_sub = this->create_subscription<base_msgs::msg::MissionStatus>(
      "/robot_master/mission_status", 10,
      std::bind(&Pcl_Clustering::master_cb, this, std::placeholders::_1));
  vision_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/cam_object_info", 10,
      std::bind(&Pcl_Clustering::vision_cb, this, std::placeholders::_1));
  // pub
  cluster_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/cluster_points", 10);
  master_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/follow_data", 10);

  // initial setting
  filt_voxel = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  colored_cluster = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

  vision_msg.data.clear();
  master_msg.auto_operation = 0;
  master_msg.mode.data = " ";
  master_msg.mission.data = " ";
  master_msg.mani_en = 0;
  master_msg.lidar_en = 0;
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

void Pcl_Clustering::vision_cb(
    const std_msgs::msg::Float32MultiArray::ConstSharedPtr& msg) {
  if (msg->data[2] == 0.0 && msg->data[3] == 0.0) {
    check_none = true;
  } else {
    check_none = false;
  }
  vision_msg.data.clear();
  vision_msg = *msg;
}

void Pcl_Clustering::master_cb(
    const base_msgs::msg::MissionStatus::ConstSharedPtr& msg) {
  master_msg = *msg;
}

void Pcl_Clustering::match_point() {
  if (check_none) {
    std_msgs::msg::Float32MultiArray pub_msg;
    pub_msg.data.push_back(0.0);
    pub_msg.data.push_back(0.5); //인식안되면 천천히 전진
    //pub_msg.data.push_back(2.0); //정지

    master_pub->publish(pub_msg);
    return;
  }
  if (vision_msg.data.empty()) {
    
    //RCLCPP_INFO(this->get_logger(), "vision_data is empty");
    return;
  }

  double du = vision_msg.data[2]; // - cam_intrinsic[2];  // 중심점 기준 X 오프셋
  double dv = vision_msg.data[3]; // - cam_intrinsic[3];  // 중심점 기준 Y 오프셋

  double theta_h = -std::atan2(du, cam_intrinsic[0]);  // 수평각 // 라이다, 카메라 좌우반대라 -
  double theta_v = std::atan2(dv, cam_intrinsic[1]);  // 수직각

  clustering();

  int clust_id = 999999;
  float min_dist = std::numeric_limits<float>::max();
  float min_angle_h = 0.0;
  float min_real_dist;
  for (size_t i = 0; i < cluster_centroids.size(); i++) {
    auto [point, angle_h, angle_v, dist, id] = cluster_centroids[i];
    float dh = theta_h - angle_h;
    float dv = theta_v - angle_v;
    float distP = std::sqrt(dh * dh + dv * dv);  // 각도 공간에서 거리
    if (distP < min_dist && dist<3) { // 각공간이 제일 가까우면서도 중심거리가 6미터 이내인 클러스터만
      min_dist = distP;
      min_real_dist = dist;  // 가장가까운각을 가진 점의 거리값
      min_angle_h = angle_h * 180 / 3.141592;
      clust_id = id;
    }
  }
  theta_h = theta_h * 180 / 3.141592;
  std::cout << "theta_h : " << theta_h << std::endl;
  coloring_points(clust_id);

  std_msgs::msg::Float32MultiArray pub_msg;
  pub_msg.data.push_back(min_angle_h);
  pub_msg.data.push_back(min_real_dist);

  master_pub->publish(pub_msg);
  pub_cluster();
}

void Pcl_Clustering::coloring_points(int clust_id) {
  for (size_t a = 0; a < colored_cluster->points.size(); a++) {
    if (colored_cluster->points[a].r == clust_id) {
      colored_cluster->points[a].r = 255;
      colored_cluster->points[a].g = 0;
      colored_cluster->points[a].b = 0;
    } else {
      colored_cluster->points[a].r = 255;
      colored_cluster->points[a].g = 255;
      colored_cluster->points[a].b = 255;
    }
  }
}

void Pcl_Clustering::clustering() {
  float angle_h = 0.0;
  float angle_v = 0.0;
  float dist = 0.0;
  colored_cluster->points.clear();

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
  int cluster_id = 0;
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
    for (int idx : indices.indices) {
      pcl::PointXYZRGB pt_rgb;
      pt_rgb.x = filt_voxel->points[idx].x;
      pt_rgb.y = filt_voxel->points[idx].y;
      pt_rgb.z = filt_voxel->points[idx].z;

      // 예: cluster_id에 따라 색 지정
      pt_rgb.r = cluster_id;
      pt_rgb.g = cluster_id;
      pt_rgb.b = cluster_id;

      colored_cluster->points.push_back(pt_rgb);
    }
    float pt_x = x_sum / n;
    float pt_y = y_sum / n;
    float pt_z = z_sum / n;

    pcl::PointXYZ centroid{pt_x, pt_y, pt_z};
    angle_h = std::atan2(pt_y, pt_x);
    angle_v = std::atan2(pt_z, std::sqrt(pt_x * pt_x + pt_y * pt_y));

    dist = std::sqrt(centroid.x * centroid.x + centroid.y * centroid.y +
                     centroid.z * centroid.z);

    cluster_centroids.push_back(
        std::make_tuple(centroid, angle_h, angle_v, dist, cluster_id));

    cluster_id++;
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
  // rclcpp::spin_some(clustering_node);
  //  RCLCPP_INFO(clustering_node->get_logger(), "setup");
  //  clustering_node->setup();

  while (rclcpp::ok()) {
    rclcpp::spin_some(clustering_node);

    clustering_node->match_point();

    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
