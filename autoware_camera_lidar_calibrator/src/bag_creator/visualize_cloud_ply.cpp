#include <iostream>

#include <ros/ros.h>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

typedef pcl::PointXYZI Point;
typedef pcl::PointCloud<Point> PointCloud;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visualize_cloud_ply");
  ros::NodeHandle nh;

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  if (args.size() != 2)
  {
    ROS_INFO("Usage: rosrun autoware_camera_lidar_calibrator "
             "visualize_cloud_ply ply_file");
    return 1;
  }

  string cloud_file = argv[1];

  // read
  pcl::PLYReader pclReader;
  PointCloud::Ptr cloud(new PointCloud());
  pclReader.read(cloud_file, *cloud);

  // visualization
  pcl::visualization::PCLVisualizer vis("vis");
  pcl::visualization::PointCloudColorHandlerCustom<Point> color_handler(
      cloud, 0.0, 0.0, 255.0);

  vis.addPointCloud(cloud, color_handler, "cloud");
  vis.spin();

  return 0;
}
