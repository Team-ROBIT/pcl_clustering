# pcl_clustering

cluster 3D points with PCL Library in ROS2

## Development Environment

| Component   | Version          |
|-------------|------------------|
| **OS**      | Ubuntu 22.04     |
| **ROS**     | Humble Hawksbill     |


## Prerequisites
Make sure to install the required package:
```
sudo apt update
sudo apt install libpcl-dev ros-humble-pcl-conversions ros-humble-pcl-msgs pcl-tools
```

## How to Use
1) Modify the parameter file and connect it to the point cloud data.
2) In RViz, run the following command and check the output.
```
ros2 launch pcl_clustering PCL_Clustering_launch.py
```
