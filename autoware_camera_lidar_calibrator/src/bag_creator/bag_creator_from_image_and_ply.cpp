#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl_ros/point_cloud.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

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

static const string lidarFrame = "velodyne_points";
static const string lidarTopic = "points_raw";
static const string cameraFrame = "cam";
static const string imageTopic = "image_raw";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bag_creator_from_image_and_ply");
  ros::NodeHandle nh;

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  if (args.size() < 2)
  {
    ROS_INFO("Usage: rosrun autoware_camera_lidar_calibrator "
             "bag_creator_from_image_and_ply data_folder");
    return 1;
  }

  // input files
  string dataFolder = args[1];
  string imageFilePath = dataFolder + "/image.jpg";
  string calibPclFilePath = dataFolder + "/calib.ply";
  string evalPclFilePath = dataFolder + "/eval.ply";

  // read image
  Mat image = imread(imageFilePath);

  if (image.empty())
  {
    ROS_INFO("Failed to open image: %s", imageFilePath.c_str());
    return 1;
  }

  // read pcl
  pcl::PLYReader pclReader;
  pcl::PointCloud<pcl::PointXYZRGB> inCalibCloud, inEvalCloud;
  pclReader.read(calibPclFilePath, inCalibCloud);
  pclReader.read(evalPclFilePath, inEvalCloud);

  // convert to type of XYZI
  PointCloud convertCalibCloud, convertEvalCloud;
  pcl::PointCloudXYZRGBtoXYZI(inCalibCloud, convertCalibCloud);
  pcl::PointCloudXYZRGBtoXYZI(inEvalCloud, convertEvalCloud);

  // set info of the cloud
  ros::Time ts = ros::Time::now();
  convertCalibCloud.header.frame_id = lidarFrame;
  convertCalibCloud.header.stamp = ts.toNSec() / 1e3;

  convertEvalCloud.header = convertCalibCloud.header;

  // write to bag file
  rosbag::Bag calibBag, evalBag;
  calibBag.open(dataFolder + "/calib.bag", rosbag::bagmode::Write);
  evalBag.open(dataFolder + "/eval.bag", rosbag::bagmode::Write);

  // write image data
  std_msgs::Header header;
  header.frame_id = cameraFrame;
  header.stamp = ts;
  sensor_msgs::ImagePtr imageMsg =
      cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
  calibBag.write(imageTopic, ts, imageMsg);
  evalBag.write(imageTopic, ts, imageMsg);

  // write pcl data
  calibBag.write(lidarTopic, ts, convertCalibCloud);
  evalBag.write(lidarTopic, ts, convertEvalCloud);

  calibBag.close();
  evalBag.close();

  cout << "Finished!" << endl;

  return 0;
}
