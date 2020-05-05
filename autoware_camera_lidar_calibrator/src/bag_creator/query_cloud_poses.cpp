#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include <iomanip>

#include <ros/ros.h>

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

void readLidarStamps(ifstream& file, vector<uint64_t>& lidar_stamps);
void readGpsPoses(ifstream& file, vector<uint64_t>& gps_stamps,
                  vector<Matrix4d>& gps_poses);
void splitEigenTransform(const Matrix4d& T, double& x, double& y, double& z,
                         double& roll, double& pitch, double& yaw);
void interpolatePose(const Matrix4d& T_prev, const Matrix4d& T_cur,
                     double factor, Matrix4d& T_inter);

bool isFileNotOpenedOrEmpty(ifstream& dataFile)
{
  return !dataFile.is_open() ||
         dataFile.peek() == std::ifstream::traits_type::eof();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "query_cloud_poses");
  ros::NodeHandle nh;

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  ros::Duration(2.0).sleep();

  if (args.size() != 3)
  {
    ROS_INFO("Usage: rosrun autoware_camera_lidar_calibrator "
             "query_local_clouds lidar.timestamps gps_poses.txt");
    return 1;
  }

  // open files
  ifstream lidarStampsFile(argv[1]);
  ifstream gpsPosesFile(argv[2]);

  cout << argv[1] << endl;
  cout << argv[2] << endl;

  if (isFileNotOpenedOrEmpty(lidarStampsFile) ||
      isFileNotOpenedOrEmpty(gpsPosesFile))
  {
    cerr << "Input data file(s) are not valid!" << endl;
    return 1;
  }

  // read lidar timestamps
  vector<uint64_t> lidarStamps;
  readLidarStamps(lidarStampsFile, lidarStamps);
  cout << "Num of lidar stamps: " << lidarStamps.size() << endl;

  // read gps poses
  vector<uint64_t> gpsStamps;
  vector<Matrix4d> gpsPoses;
  readGpsPoses(gpsPosesFile, gpsStamps, gpsPoses);
  assert(gpsStamps.size() == gpsPoses.size());
  cout << "Num of gps poses: " << gpsPoses.size() << endl;

  // get interpolated lidar gps poses
  vector<Matrix4d> lidarGpsPoses;
  vector<uint64_t> validLidarStamps;
  uint64_t curGpsStampInd = 0;
  bool finished = false;
  for (auto& t : lidarStamps)
  {
    // jump to the valid lidar timestamp
    if (t < gpsStamps[curGpsStampInd])
      continue;

    // jump to the appropriate gps stamps index
    while (t > gpsStamps[curGpsStampInd])
    {
      curGpsStampInd++;
      if (curGpsStampInd == gpsStamps.size())
      {
        finished = true;
        break;
      }
    }

    // check if finished
    if (finished || !ros::ok())
      break;

    // check if there exists one pose on both sides of current lidar stamp
    if (curGpsStampInd > 1)
    {
      // get the interpolated factor
      uint64_t preStamp = gpsStamps[curGpsStampInd - 1];
      uint64_t curStamp = gpsStamps[curGpsStampInd];
      double factor = ((double)t - (double)preStamp) /
                      ((double)curStamp - (double)preStamp);

      // get the interpolated pose
      Matrix4d prePose = gpsPoses[curGpsStampInd - 1];
      Matrix4d curPose = gpsPoses[curGpsStampInd];

      Matrix4d interPose = Matrix4d::Identity();
      interpolatePose(prePose, curPose, factor, interPose);

      // add to buffer
      validLidarStamps.push_back(t);
      lidarGpsPoses.push_back(interPose);
    }
  }

  assert(validLidarStamps.size() == lidarGpsPoses.size());

  // write to file
  string outputFilePath = argv[1];
  size_t found = outputFilePath.find_last_of('/');
  if (found != std::string::npos)
  {
    outputFilePath =
        outputFilePath.substr(0, found + 1) + "lidar_gps_poses.txt";
  }

  ofstream outputFile(outputFilePath.c_str());
  outputFile << "# lidar_stamp x y z roll pitch yaw" << endl;
  for (size_t i = 0; i < lidarStamps.size(); i++)
  {
    double x, y, z, roll, pitch, yaw;
    splitEigenTransform(lidarGpsPoses[i], x, y, z, roll, pitch, yaw);
    outputFile << validLidarStamps[i] << std::fixed << std::setprecision(6)
               << ' ' << x << ' ' << y << ' ' << z << ' ' << roll << ' '
               << pitch << ' ' << yaw << ' ' << endl;
  }

  cout << "Finished!" << endl;

  return 0;
}

void readLidarStamps(ifstream& file, vector<uint64_t>& lidar_stamps)
{
  uint64_t tmpStamp;
  while (file >> tmpStamp)
    lidar_stamps.push_back(tmpStamp);
}

void readGpsPoses(ifstream& file, vector<uint64_t>& gps_stamps,
                  vector<Matrix4d>& gps_poses)
{
  string line;
  while (getline(file, line))
  {
    if (line[0] == '#')
      continue;

    stringstream lineStr(line);

    uint64_t timestamp;

    double yaw, pitch, roll, t_x, t_y, t_z;
    lineStr >> timestamp >> t_x >> t_y >> t_z >> roll >> pitch >> yaw;

    Matrix3d R;
    R = AngleAxisd(yaw, Vector3d::UnitZ()) *
        AngleAxisd(pitch, Vector3d::UnitY()) *
        AngleAxisd(roll, Vector3d::UnitX());

    Vector3d T(t_x, t_y, t_z);

    Matrix4d curPose = Matrix4d::Identity();
    curPose.topLeftCorner(3, 3) = R;
    curPose.topRightCorner(3, 1) = T;

    gps_stamps.push_back(timestamp);
    gps_poses.push_back(curPose);
  }
}

void splitEigenTransform(const Matrix4d& T, double& x, double& y, double& z,
                         double& roll, double& pitch, double& yaw)
{
  Vector3d t = T.topRightCorner(3, 1);
  x = t[0];
  y = t[1];
  z = t[2];

  Matrix3d R = T.topLeftCorner(3, 3);
  Vector3d ea = R.eulerAngles(2, 1, 0);
  roll = ea[2];
  pitch = ea[1];
  yaw = ea[0];
}

void interpolatePose(const Matrix4d& T_prev, const Matrix4d& T_cur,
                     double factor, Matrix4d& T_inter)
{
  // interpolate translation
  Vector3d t_prev = T_prev.topRightCorner(3, 1);
  Vector3d t_cur = T_cur.topRightCorner(3, 1);

  Vector3d t_inter = t_prev + factor * (t_cur - t_prev);

  // interpolate rotation
  Matrix3d R_prev = T_prev.topLeftCorner(3, 3);
  Matrix3d R_cur = T_cur.topLeftCorner(3, 3);

  Quaterniond quat_prev(R_prev);
  Quaterniond quat_cur(R_cur);
  Quaterniond quat_inter = quat_prev.slerp(factor, quat_cur);

  Matrix3d R_inter = quat_inter.toRotationMatrix();

  // combine
  T_inter.topRightCorner(3, 1) = t_inter;
  T_inter.topLeftCorner(3, 3) = R_inter;
}
