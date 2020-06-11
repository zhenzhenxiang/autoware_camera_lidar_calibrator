#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types_conversion.h>

#include <opencv2/opencv.hpp>

typedef pcl::PointXYZI Point;
typedef pcl::PointCloud<Point> PointCloud;

using namespace std;
using namespace ros;
using namespace cv;

static const string lidarFrame = "velodyne";
static const string lidarTopic = "/velodyne_points";
static const string cameraFrame = "cam";
static const string imageTopic = "/image_raw";

/*
* A point cloud type that has "ring" channel
*/
struct PointXYZIR
{
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIR,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                            intensity)(uint16_t, ring, ring))

bool isFileNotOpenedOrEmpty(ifstream& dataFile)
{
  return !dataFile.is_open() ||
         dataFile.peek() == std::ifstream::traits_type::eof();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "create_bag_from_ply");
  ros::NodeHandle nh;

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  if (args.size() < 3)
  {
    ROS_INFO("Usage: rosrun autoware_camera_lidar_calibrator "
             "create_bag_from_ply clouds.txt output.bag [has_ring_info]");
    return 1;
  }

  // check if has ring index
  bool has_ring_info = false;
  if (args.size() == 4)
    has_ring_info = true;

  // input cloud file
  string cloudPaths = args[1];
  ifstream cloudPathFile(cloudPaths.c_str());

  if (isFileNotOpenedOrEmpty(cloudPathFile))
  {
    cerr << "Input cloud data file is not valid!" << endl;
    return 1;
  }

  // output bag file
  string outputPath = argv[2];
  rosbag::Bag cloudBag;
  cloudBag.open(outputPath, rosbag::bagmode::Write);

  // read pcl and write to bag
  pcl::PLYReader pclReader;
  string cloudFile;

  while (cloudPathFile >> cloudFile)
  {
    // get cloud timestamp from file name
    uint64_t stampPCL = atoll(
        cloudFile.substr(cloudFile.size() - 20, cloudFile.size() - 4).c_str());

    ros::Time stampROS;
    stampROS.fromNSec(stampPCL * 1e3);

    if (has_ring_info)
    {
      pcl::PointCloud<PointXYZIR> inCloudRing;
      pclReader.read(cloudFile, inCloudRing);
      // set info of the cloud
      inCloudRing.header.frame_id = lidarFrame;
      inCloudRing.header.stamp = stampPCL;

      // write pcl data
      cloudBag.write(lidarTopic, stampROS, inCloudRing);
    }
    else
    {
      PointCloud inCloud;
      pclReader.read(cloudFile, inCloud);

      // set info of the cloud
      inCloud.header.frame_id = lidarFrame;
      inCloud.header.stamp = stampPCL;

      // write pcl data
      cloudBag.write(lidarTopic, stampROS, inCloud);
    }
  }

  cloudBag.close();

  cout << "Finished!" << endl;

  return 0;
}
