/*
 * camera_lidar_calibration.cpp
 *
 *  Created on: Sep 4, 2017
 *      Author: amc-jp
 */

#include <string>
#include <vector>
#include <ctime>
#include <sstream>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <autoware_msgs/projection_matrix.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <boost/filesystem.hpp>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#if (CV_MAJOR_VERSION == 3)
  #include <opencv2/imgcodecs.hpp>
#else
  #include <opencv2/contrib/contrib.hpp>
#endif

#define __APP_NAME__ "autoware_camera_lidar_calibration_node"

class RosCameraLidarApp

{
  ros::NodeHandle 	node_handle_;

  ros::Subscriber 	subscriber_image_raw_;
  ros::Subscriber 	subscriber_points_raw_;
  ros::Subscriber     subscriber_intrinsics_;
  ros::Subscriber     subscriber_extrinsics_;
  ros::Subscriber     subscriber_clicked_point_;
  ros::Subscriber     subscriber_image_point_;

  cv::Size            image_size_;
  cv::Mat             camera_instrinsics_;
  cv::Mat             distortion_coefficients_;
  cv::Mat             camera_extrinsics_;

  cv::Mat             lidar_camera_rotation_matrix_;
  cv::Mat             lidar_camera_translation_vector_;

  std::vector<cv::Point2f> clicked_image_points_;
  std::vector<cv::Point3f> clicked_velodyne_points_;


  bool is_rotation_matrix(const cv::Mat& in_matrix)
  {
    cv::Mat rotation_matrix;
    transpose(in_matrix, rotation_matrix);

    cv::Mat test_matrix = rotation_matrix * in_matrix;
    cv::Mat test_result = cv::Mat::eye(3,3, test_matrix.type());

    return cv::norm(test_result, test_matrix) < 1e-6;

  }

  cv::Point3f get_rpy_from_matrix(const cv::Mat& in_rotation)
  {

    assert(is_rotation_matrix(in_rotation));

    float sy = sqrt(in_rotation.at<double>(0,0) * in_rotation.at<double>(0,0) +
                        in_rotation.at<double>(1,0) * in_rotation.at<double>(1,0) );

    bool is_singular = sy < 1e-6;

    float x, y, z;
    if (!is_singular)
    {
      x = atan2(in_rotation.at<double>(2,1), in_rotation.at<double>(2,2));
      y = atan2(-in_rotation.at<double>(2,0), sy);
      z = atan2(in_rotation.at<double>(1,0), in_rotation.at<double>(0,0));
    }
    else
    {
      x = atan2(-in_rotation.at<double>(1,2), in_rotation.at<double>(1,1));
      y = atan2(-in_rotation.at<double>(2,0), sy);
      z = 0;
    }
    return cv::Point3f(x, y, z);
  }

  void SaveCalibrationFile(cv::Mat in_extrinsic, cv::Mat in_intrinsic, cv::Mat in_dist_coeff, cv::Size in_size, float error)
  {
    std::string path_filename_str;
    const char *homedir;

    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,sizeof(buffer),"%Y%m%d_%H%M%S",timeinfo);
    std::string datetime_str(buffer);

    if ((homedir = getenv("HOME")) == NULL) {
      homedir = getpwuid(getuid())->pw_dir;
    }

    path_filename_str = std::string(homedir) + "/"+datetime_str+"_autoware_lidar_camera_calibration.yaml";

    cv::FileStorage fs(path_filename_str.c_str(), cv::FileStorage::WRITE);
    if(!fs.isOpened()){
      fprintf(stderr, "%s : cannot open file\n", path_filename_str.c_str());
      exit(EXIT_FAILURE);
    }

    fs << "CameraExtrinsicMat" << in_extrinsic;
    fs << "CameraMat" << in_intrinsic;
    fs << "DistCoeff" << in_dist_coeff;
    fs << "ImageSize" << in_size;
    fs << "ReprojectionError" << error;
    fs << "DistModel" << "plumb_bob";

    cv::Mat calib_data = cv::Mat::zeros(clicked_image_points_.size(), 5, CV_32F);
    cv::Mat tmp = cv::Mat::zeros(1, 5, CV_32F);
    for (unsigned int i = 0; i < clicked_image_points_.size(); i++)
    {
      tmp.at<float>(0, 0) = clicked_image_points_[i].x;
      tmp.at<float>(0, 1) = clicked_image_points_[i].y;
      tmp.at<float>(0, 2) = clicked_velodyne_points_[i].x;
      tmp.at<float>(0, 3) = clicked_velodyne_points_[i].y;
      tmp.at<float>(0, 4) = clicked_velodyne_points_[i].z;

      tmp.copyTo(calib_data(cv::Rect_<float>(0, i, 5, 1)));
    }
    fs << "CalibData" << calib_data;

    std::cout << "CalibData:\n" << calib_data << std::endl;

    ROS_INFO("Wrote Autoware Calibration file in: %s", path_filename_str.c_str());
  }

  void CalibrateSensors()
  {
    std::cout << "Number of points: " << clicked_velodyne_points_.size() << ":"  << clicked_image_points_.size() << std::endl;

    if (clicked_velodyne_points_.size() == clicked_image_points_.size()
        && clicked_image_points_.size() > 8)
    {
      // Extrinsics from camera to lidar
      cv::Mat R_cam2lidar = camera_extrinsics_(cv::Rect_<double>(0, 0, 3, 3));
      cv::Mat t_cam2lidar = camera_extrinsics_(cv::Rect_<double>(3, 0, 1, 3));

      // Extrinsics from lidar to camera for solving PnP
      cv::Mat rotation_vector, translation_vector;
      cv::Rodrigues(R_cam2lidar.t(), rotation_vector);
      translation_vector = -R_cam2lidar.t() * t_cam2lidar;

      cv::solvePnP(clicked_velodyne_points_,
                   clicked_image_points_,
                   camera_instrinsics_,
                   distortion_coefficients_,
                   rotation_vector,
                   translation_vector,
                   true,
                   CV_ITERATIVE
      );

      cv::Mat rotation_matrix;
      cv::Rodrigues(rotation_vector, rotation_matrix);

      // Calculate reprojection errors
      std::vector<cv::Point2f> projected_image_points;
      cv::projectPoints(clicked_velodyne_points_, rotation_vector, translation_vector, camera_instrinsics_,
                        distortion_coefficients_, projected_image_points);

      float reprojection_error = 0.0;
      for (unsigned int i = 0; i < clicked_image_points_.size(); i++)
      {
        float error_x = clicked_image_points_[i].x - projected_image_points[i].x;
        float error_y = clicked_image_points_[i].y - projected_image_points[i].y;
        reprojection_error += sqrt(error_x * error_x + error_y * error_y);
      }
      reprojection_error /= clicked_image_points_.size();

      std::cout << "Reprojection_error: " << reprojection_error << std::endl;

      // Get estimated extrinsics from camera to lidar
      R_cam2lidar = rotation_matrix.t();
      t_cam2lidar = -rotation_matrix.t() * translation_vector;

      std::cout << "Extrinsics from camera to lidar:" << std::endl;
      std::cout << "Rotation:\n" << R_cam2lidar << std::endl << std::endl;
      std::cout << "Translation:\n" << t_cam2lidar << std::endl << std::endl;
      std::cout << "RPY:\n" << get_rpy_from_matrix(R_cam2lidar) << std::endl << std::endl;

      R_cam2lidar.copyTo(camera_extrinsics_(cv::Rect_<double>(0, 0, 3, 3)));
      t_cam2lidar.copyTo(camera_extrinsics_(cv::Rect_<double>(3, 0, 1, 3)));

      // std::cout << "extrinsics: \n" << extrinsics << std::endl;

      SaveCalibrationFile(camera_extrinsics_,camera_instrinsics_, distortion_coefficients_, image_size_, reprojection_error);
    }
  }

  void RvizClickedPointCallback(const geometry_msgs::PointStamped& in_clicked_point)
  {
    clicked_velodyne_points_.push_back(cv::Point3f(in_clicked_point.point.x,
                                                   in_clicked_point.point.y,
                                                   in_clicked_point.point.z));
    std::cout << cv::Point3f(in_clicked_point.point.x,
                             in_clicked_point.point.y,
                             in_clicked_point.point.z) << std::endl << std::endl;
    CalibrateSensors();
  }
  void ImageClickedPointCallback(const geometry_msgs::PointStamped& in_clicked_point)
  {

    clicked_image_points_.push_back(cv::Point2f(in_clicked_point.point.x,
                                                   in_clicked_point.point.y));

    std::cout << cv::Point2f(in_clicked_point.point.x,
                             in_clicked_point.point.y) << std::endl << std::endl;

    CalibrateSensors();
  }

  void ImageCallback(const sensor_msgs::Image& in_image_sensor)
  {

    if (camera_instrinsics_.empty() || distortion_coefficients_.empty())
    {
      ROS_INFO("[%s] No Camera intrinsics loaded. Make sure to publish and set correctly camera_info source topic.", __APP_NAME__);
      return;
    }
  }

  void IntrinsicsCallback(const sensor_msgs::CameraInfo& in_message)
  {
    if (camera_instrinsics_.empty() || distortion_coefficients_.empty())
    {
      image_size_.height = in_message.height;
      image_size_.width = in_message.width;

      camera_instrinsics_ = cv::Mat(3, 3, CV_64F);
      for (int row = 0; row < 3; row++)
      {
        for (int col = 0; col < 3; col++)
        {
          camera_instrinsics_.at<double>(row, col) = in_message.K[row * 3 + col];
        }
      }

      distortion_coefficients_ = cv::Mat(1, 5, CV_64F);
      for (int col = 0; col < 5; col++)
      {
        distortion_coefficients_.at<double>(col) = in_message.D[col];
      }
      ROS_INFO("[%s] Camera Intrinsics Loaded", __APP_NAME__);
    }
  }

  void ExtrinsicsCallback(const autoware_msgs::projection_matrix& in_message)
  {
    if (camera_extrinsics_.empty())
    {
      camera_extrinsics_ = cv::Mat(4, 4, CV_64F);
      for (int row = 0; row < 4; row++)
      {
        for (int col = 0; col < 4; col++)
        {
          camera_extrinsics_.at<double>(row, col) = in_message.projection_matrix[row * 4 + col];
        }
      }
      std::cout << "Extrinsics: \n" << camera_extrinsics_ << std::endl;
      ROS_INFO("[%s] Camera Extrinsics Loaded", __APP_NAME__);
    }
  }

public:
  void Run()
  {
    ros::NodeHandle private_node_handle("~");//to receive args

    std::string image_raw_topic_str, camera_info_topic_str, camera_extrisics_topic_str;
    std::string name_space_str = ros::this_node::getNamespace();

    private_node_handle.param<std::string>("image_src", image_raw_topic_str, "/image_rectified");
    private_node_handle.param<std::string>("camera_info_src", camera_info_topic_str, "camera_info");
    private_node_handle.param<std::string>("camera_extrisics_src", camera_extrisics_topic_str, "projection_matrix");

    if (name_space_str != "/")
    {
      if (name_space_str.substr(0, 2) == "//")
      {
        name_space_str.erase(name_space_str.begin());
      }
      image_raw_topic_str = name_space_str + image_raw_topic_str;
      camera_info_topic_str = name_space_str + camera_info_topic_str;
      camera_extrisics_topic_str = name_space_str + camera_extrisics_topic_str;
    }

    ROS_INFO("[%s] image_src: %s",__APP_NAME__, image_raw_topic_str.c_str());
    ROS_INFO("[%s] camera_info_src: %s",__APP_NAME__, camera_info_topic_str.c_str());
    ROS_INFO("[%s] camera_extrinsics_src: %s",__APP_NAME__, camera_extrisics_topic_str.c_str());


    ROS_INFO("[%s] Subscribing to... %s",__APP_NAME__, image_raw_topic_str.c_str());
    subscriber_image_raw_ = node_handle_.subscribe(image_raw_topic_str, 1, &RosCameraLidarApp::ImageCallback, this);


    ROS_INFO("[%s] Subscribing to... %s",__APP_NAME__, camera_info_topic_str.c_str());
    subscriber_intrinsics_ = node_handle_.subscribe(camera_info_topic_str, 1, &RosCameraLidarApp::IntrinsicsCallback, this);

    subscriber_extrinsics_ = node_handle_.subscribe(camera_extrisics_topic_str, 1, &RosCameraLidarApp::ExtrinsicsCallback, this);

    ROS_INFO("[%s] Subscribing to PointCloud ClickedPoint from RVIZ... /clicked_point",__APP_NAME__);
    subscriber_clicked_point_ = node_handle_.subscribe("/clicked_point", 1, &RosCameraLidarApp::RvizClickedPointCallback, this);

    ROS_INFO("[%s] Subscribing to Image ClickedPoint from JSK ImageView2... %s/screenpoint",__APP_NAME__, image_raw_topic_str.c_str());
    subscriber_image_point_ = node_handle_.subscribe(image_raw_topic_str+"/screenpoint", 1, &RosCameraLidarApp::ImageClickedPointCallback, this);
    ROS_INFO("[%s] ClickedPoint: %s",__APP_NAME__, (image_raw_topic_str+"/screenpoint").c_str());

    ROS_INFO("[%s] Ready. Waiting for data...",__APP_NAME__);
    ros::spin();
    ROS_INFO("[%s] END",__APP_NAME__);
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, __APP_NAME__);

  RosCameraLidarApp app;

  app.Run();

  return 0;
}
