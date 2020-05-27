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

  // get cloud range
  Point minPoint, maxPoint;
  pcl::getMinMax3D(*cloud, minPoint, maxPoint);
  cout << "Cloud range:\n"
       << "X: " << minPoint.x << ' ' << maxPoint.x << endl
       << "Y: " << minPoint.y << ' ' << maxPoint.y << endl
       << "Z: " << minPoint.z << ' ' << maxPoint.z << endl;

  double minIntensity = 1e6, maxIntensity = -1e6;

  for (auto p : cloud->points)
  {
    if (p.intensity < minIntensity)
      minIntensity = p.intensity;

    if (p.intensity > maxIntensity)
      maxIntensity = p.intensity;
  }
  cout << "Intensity: " << minIntensity << ' ' << maxIntensity << endl;

  if (minIntensity < 0)
  {
    cout << "remove negative intensities..." << endl;
    for (auto& p : cloud->points)
      if (p.intensity < 0)
        p.intensity = 0;
  }

  // get cloud fields
  vector<pcl::PCLPointField> fields;
  pcl::getFields(*cloud, fields);
  for (auto& f : fields)
    cout << "fields:\n" << f << endl;

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

  pcl::visualization::PointCloudColorHandlerGenericField<Point> color_handler(
      cloud, fields[field_ind].name);
  vis->addPointCloud(cloud, color_handler, "cloud");

  cout << "current color field: " << fields[field_ind].name << endl;

  vis->registerKeyboardCallback(keyboardEventOccurred);
  cout << "Press 's' to switch color field!" << endl;

  while (!vis->wasStopped())
  {
    if (switch_color_field)
    {
      // update field index
      ++field_ind;
      if (field_ind == fields.size())
        field_ind = 0;

      // update cloud color
      pcl::visualization::PointCloudColorHandlerGenericField<Point>
          color_handler(cloud, fields[field_ind].name);
      vis->updatePointCloud(cloud, color_handler, "cloud");

      cout << "switch to color field: " << fields[field_ind].name << endl;

      // reset flag
      switch_color_field = false;
    }

    vis->spinOnce(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}
