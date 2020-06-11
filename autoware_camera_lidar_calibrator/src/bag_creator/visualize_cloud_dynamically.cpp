#include <iostream>
#include <thread>

#include <ros/ros.h>

#include <pcl/common/common.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter.h>

using namespace std;

typedef pcl::PointXYZI Point;
typedef pcl::PointCloud<Point> PointCloud;

int field_num = 0;
size_t field_ind = 0;

bool switch_color_field = false;

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event)
{
  if (event.getKeySym() == "s" && event.keyDown())
  {
    switch_color_field = true;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visualize_cloud_dynamically");
  ros::NodeHandle nh;

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  if (args.size() != 2)
  {
    ROS_INFO("Usage: rosrun autoware_camera_lidar_calibrator "
             "visualize_cloud_dynamically ply_file");
    return 1;
  }

  string cloud_file = argv[1];

  // read
  pcl::PLYReader pclReader;
  PointCloud::Ptr cloud(new PointCloud());
  pclReader.read(cloud_file, *cloud);

  // get cloud info
  cout << "cloud points num: " << cloud->size() << endl
       << "width: " << cloud->width << endl
       << "height: " << cloud->height << endl
       << "is_dense: " << cloud->is_dense << endl
       << endl;

  // remove NaN
  std::vector<int> indices;
  cloud->is_dense = false; // needed for the following function
  pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

  cout << "cloud points num after NaN removed: " << cloud->size() << endl;

  // visualization
  pcl::visualization::PCLVisualizer::Ptr vis(
      new pcl::visualization::PCLVisualizer("3D Viewer"));

  vis->addCoordinateSystem(5.0);

  // -- set camera as the bird's-eye view
  vis->initCameraParameters();
  vis->setCameraPosition(0.0, 0.0, 50.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0);
  pcl::visualization::Camera camera;
  vis->getCameraParameters(camera);
  cout << "Camera params:" << endl;
  cout << "pos: " << camera.pos[0] << ' ' << camera.pos[1] << ' '
       << camera.pos[2] << endl;
  cout << "focal('view point' in visualizer): " << camera.focal[0] << ' '
       << camera.focal[1] << ' ' << camera.focal[2] << endl;
  cout << "view('view up' in visualizer): " << camera.view[0] << ' '
       << camera.view[1] << ' ' << camera.view[2] << endl;

  PointCloud::Ptr cloud_for_vis(new PointCloud());

  pcl::visualization::PointCloudColorHandlerCustom<Point> color_handler(
      cloud_for_vis, 0.0, 255.0, 0.0);
  vis->addPointCloud(cloud_for_vis, color_handler, "cloud");

  int point_index = 0;

  while (!vis->wasStopped())
  {
    if (cloud_for_vis->size() != cloud->size())
    {
      // add point for visualization
      cloud_for_vis->push_back(cloud->points[point_index]);
      ++point_index;

      // update cloud
      vis->updatePointCloud(cloud_for_vis, color_handler, "cloud");
    }
    else
    {
      // reset to show again
      cloud_for_vis->clear();
      point_index = 0;
    }

    vis->spinOnce(10);
  }

  return 0;
}
