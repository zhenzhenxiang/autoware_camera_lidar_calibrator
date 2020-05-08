#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>

#include <ros/ros.h>

#include <opencv2/opencv.hpp>

#include <Eigen/Dense>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types_conversion.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pclomp/ndt_omp.h>

using namespace std;
using namespace Eigen;

typedef pcl::PointXYZI Point;
typedef pcl::PointCloud<Point> PointCloud;

void readLidarGpsExtrinsicParam(ifstream& file, Matrix4d& T_lidar2gps);
void readLidarGpsPoses(ifstream& file, vector<uint64_t>& timestamps,
                       vector<Matrix4d>& lidar_gps_poses);
bool findTimestampIndex(const uint64_t stamp,
                        const vector<uint64_t>& timestamps, size_t& index);
void cloudRangeFilter(const PointCloud::ConstPtr& inCloud, double minRange,
                      double maxRange, PointCloud::Ptr& outCloud);

bool isFileNotOpenedOrEmpty(ifstream& dataFile)
{
  return !dataFile.is_open() ||
         dataFile.peek() == std::ifstream::traits_type::eof();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "query_local_clouds");
  ros::NodeHandle nh;

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  ros::Duration(3.0).sleep();

  if (args.size() != 4)
  {
    ROS_INFO("Usage: rosrun autoware_camera_lidar_calibrator "
             "query_local_clouds query_time pose_range data_folder");
    return 1;
  }

  uint64_t queryTimestamp = atoll(argv[1]);
  double queryPoseRange = atof(argv[2]);

  string dataFolder = argv[3];

  string cloudStampsPath = dataFolder + '/' + "lidar.timestamps";
  string cloudsFilePath = dataFolder + '/' + "clouds.txt";
  string posesFilePath = dataFolder + '/' + "lidar_gps_poses.txt";
  string calibFilePath = dataFolder + '/' + "calib.txt";

  // open files
  ifstream cloudStampsFile(cloudStampsPath.c_str());
  ifstream cloudsFile(cloudsFilePath.c_str());
  ifstream posesFile(posesFilePath.c_str());
  ifstream calibFile(calibFilePath.c_str());

  if (isFileNotOpenedOrEmpty(cloudStampsFile) ||
      isFileNotOpenedOrEmpty(cloudsFile) || isFileNotOpenedOrEmpty(posesFile) ||
      isFileNotOpenedOrEmpty(calibFile))
  {
    cerr << "Input data file(s) are not valid!" << endl;
    return 1;
  }

  // read extrinsic param from lidar to gps
  Matrix4d T_lidar2gps = Matrix4d::Identity();
  readLidarGpsExtrinsicParam(calibFile, T_lidar2gps);

  // read pcl file list
  int numPcl = 0;
  vector<string> cloudList;
  string line;
  while (getline(cloudsFile, line))
  {
    cloudList.push_back(line);
    numPcl++;
  }

  cout << numPcl << " cloud files loaded!" << endl;

  // search query timestamp
  auto iterTimestamp = find_if(
      cloudList.begin(), cloudList.end(), [&queryTimestamp](const string& file)
      {
        uint64_t curTimestamp =
            atoll(file.substr(file.size() - 20, file.size() - 4).c_str());
        return queryTimestamp == curTimestamp;
      });

  if (iterTimestamp == cloudList.end())
  {
    cout << "Cannot find the query timestamp in clouds.txt." << endl;
    return 1;
  }

  int queryIndex = iterTimestamp - cloudList.begin();

  // read poses
  vector<uint64_t> lidar_gps_timestamps;
  vector<Matrix4d> lidar_gps_poses;
  readLidarGpsPoses(posesFile, lidar_gps_timestamps, lidar_gps_poses);

  // get the pose at the query timestamp
  size_t stamp_index = 0;
  bool findQueryPose =
      findTimestampIndex(queryTimestamp, lidar_gps_timestamps, stamp_index);

  if (!findQueryPose)
  {
    cout << "Failed to find query pose corresponding to query timestamp!"
         << endl;
    return 1;
  }

  Matrix4d queryPose = lidar_gps_poses[stamp_index];

  // construct kd-tree for pose searching
  PointCloud::Ptr poseCloud(new PointCloud());
  for (auto pose : lidar_gps_poses)
  {
    Vector3d pos = pose.topRightCorner(3, 1);
    Point pt;
    pt.x = pos(0);
    pt.y = pos(1);
    pt.z = pos(2);
    poseCloud->push_back(pt);
  }

  Vector3d queryPosition = queryPose.topRightCorner(3, 1);
  Point queryPoint;
  queryPoint.x = queryPosition(0);
  queryPoint.y = queryPosition(1);
  queryPoint.z = queryPosition(2);

  pcl::KdTreeFLANN<Point> kdtree;
  kdtree.setInputCloud(poseCloud);

  // find the poses in the query range
  vector<int> pointIdxRadiusSearch;
  vector<float> dist;
  int posesInRange = kdtree.radiusSearch(queryPoint, queryPoseRange,
                                         pointIdxRadiusSearch, dist);
  cout << "Found poses in range: " << posesInRange << endl;

  // compute relative poses to query pose
  vector<Matrix4d> selectedPoses;
  for (int i = 0; i < posesInRange; i++)
  {
    int ind = pointIdxRadiusSearch[i];
    Matrix4d tmpPose = T_lidar2gps.inverse() * queryPose.inverse() *
                       lidar_gps_poses[ind] * T_lidar2gps;
    selectedPoses.push_back(tmpPose);
  }

  // read pcl data in the query range
  double minDist = 4.0;
  double maxDist = 200.0;

  pcl::PLYReader pclReader;
  vector<PointCloud::Ptr> pointClouds;
  for (int i = 0; i < posesInRange; i++)
  {
    int ind = pointIdxRadiusSearch[i];

    PointCloud::Ptr pointsRaw(new PointCloud());
    pclReader.read(cloudList[ind], *pointsRaw);

    // remove NaN
    PointCloud::Ptr pointsValid(new PointCloud());
    std::vector<int> indices;
    pointsRaw->is_dense = false; // needed for the following function
    pcl::removeNaNFromPointCloud(*pointsRaw, *pointsValid, indices);

    // apply range filter
    PointCloud::Ptr points(new PointCloud());
    cloudRangeFilter(pointsValid, minDist, maxDist, points);

    pointClouds.push_back(points);
  }

  PointCloud::Ptr queryCloudRaw(new PointCloud());
  pclReader.read(cloudList[queryIndex], *queryCloudRaw);

  // remove NaN
  PointCloud::Ptr queryCloudValid(new PointCloud());
  std::vector<int> indices;
  queryCloudRaw->is_dense = false; // needed for the following function
  pcl::removeNaNFromPointCloud(*queryCloudRaw, *queryCloudValid, indices);

  // apply range filter
  PointCloud::Ptr queryCloud(new PointCloud());
  cloudRangeFilter(queryCloudValid, minDist, maxDist, queryCloud);

  // create downsampling filter
  pcl::VoxelGrid<Point> voxelgrid;
  voxelgrid.setLeafSize(0.3f, 0.3f, 0.3f);

  // downsample target cloud
  PointCloud::Ptr downsampled(new PointCloud());
  voxelgrid.setInputCloud(queryCloud);
  voxelgrid.filter(*downsampled);
  queryCloud = downsampled;

  // create ndt aligner
  pclomp::NormalDistributionsTransform<Point, Point>::Ptr ndt_omp(
      new pclomp::NormalDistributionsTransform<Point, Point>());
  ndt_omp->setStepSize(0.1);
  ndt_omp->setResolution(1.0);

  // transform point clouds to the query frame
  vector<PointCloud::Ptr> pointCloudsAccumulated;
  vector<PointCloud::Ptr> pointCloudsAligned;
  vector<Matrix4d> alignedPoses;
  vector<Matrix4d> alignedAccPoses;

  PointCloud::Ptr queryAccCloud(new PointCloud());
  *queryAccCloud += *queryCloud;

  pcl::visualization::PCLVisualizer visAlign("visAlign");

  for (size_t i = 0; i < pointClouds.size(); i++)
  {
    // case 1: accumulate pointclouds
    PointCloud::Ptr pclAccumulated(new PointCloud());
    pcl::transformPointCloud(*pointClouds[i], *pclAccumulated,
                             selectedPoses[i].cast<float>());
    pointCloudsAccumulated.push_back(pclAccumulated);

    // case 2: align to query poincloud with NDT
    PointCloud::Ptr pclAligned(new PointCloud());

    // downsampling
    PointCloud::Ptr downsampled(new PointCloud());
    voxelgrid.setInputCloud(pointClouds[i]);
    voxelgrid.filter(*downsampled);
    pointClouds[i] = downsampled;

    // align
    Matrix4f initTransform = selectedPoses[i].cast<float>();
    ndt_omp->setInputSource(pointClouds[i]);
    ndt_omp->setInputTarget(queryCloud);
    ndt_omp->align(*pclAligned, initTransform);

    pointCloudsAligned.push_back(pclAligned);
    Matrix4f finalTransform = ndt_omp->getFinalTransformation();
    alignedPoses.push_back(finalTransform.cast<double>());

    // print status
    cout << "Aligned Cloud #" << i << endl;
    cout << "score: " << ndt_omp->getFitnessScore() << endl;
    cout << "initial transform: \n" << initTransform << endl;
    cout << "final transform: \n" << finalTransform << endl;

    // case 3: align to accumulated pointclouds with NDT
    PointCloud::Ptr pclAlignedAcc(new PointCloud());

    // downsampling
    PointCloud::Ptr downsampled2(new PointCloud());
    voxelgrid.setInputCloud(queryAccCloud);
    voxelgrid.filter(*downsampled2);
    queryAccCloud = downsampled2;

    // align
    ndt_omp->setInputTarget(queryAccCloud);
    ndt_omp->align(*pclAlignedAcc, initTransform);

    finalTransform = ndt_omp->getFinalTransformation();
    alignedAccPoses.push_back(finalTransform.cast<double>());
    *queryAccCloud += *pclAlignedAcc;

    // print status
    cout << "Aligned Accumulated Cloud #" << i << endl;
    cout << "score: " << ndt_omp->getFitnessScore() << endl;
    cout << "initial transform: \n" << initTransform << endl;
    cout << "final transform: \n" << finalTransform << endl;

    // display
    pcl::visualization::PointCloudColorHandlerCustom<Point> query_handler(
        queryAccCloud, 0.0, 255.0, 0.0);
    pcl::visualization::PointCloudColorHandlerCustom<Point> aligned_handler(
        pclAlignedAcc, 255.0, 0.0, 0.0);
    visAlign.removeAllPointClouds();
    visAlign.addPointCloud(queryAccCloud, query_handler, "query");
    visAlign.addPointCloud(pclAlignedAcc, aligned_handler, "alignedAcc");
    visAlign.spinOnce();
  }

  // merge into one point cloud
  PointCloud::Ptr mergedAccumulatedCloud(new PointCloud());
  PointCloud::Ptr mergedAlignedCloud(new PointCloud());
  for (size_t i = 1; i < pointCloudsAligned.size(); i++)
  {
    *mergedAccumulatedCloud += *pointCloudsAccumulated[i];
    *mergedAlignedCloud += *pointCloudsAligned[i];
  }

  cout << "Total points of accumulated cloud: "
       << mergedAccumulatedCloud->points.size() << endl;
  cout << "Total points of aligned cloud: " << mergedAlignedCloud->points.size()
       << endl;

  // save to ply file
  pcl::PLYWriter writer;
  string mergedAccumulatedPlyFileName = dataFolder + "/merged_accumulated_" +
                                        to_string(queryTimestamp) + '_' +
                                        to_string(queryPoseRange) + ".ply";
  writer.write(mergedAccumulatedPlyFileName, *mergedAccumulatedCloud, true);

  string mergedAlignedPlyFileName = dataFolder + "/merged_aligned_" +
                                    to_string(queryTimestamp) + '_' +
                                    to_string(queryPoseRange) + ".ply";
  writer.write(mergedAlignedPlyFileName, *mergedAlignedCloud, true);

  string mergedAlignedAccPlyFileName =
      dataFolder + "/merged_aligned_accumulated_" + to_string(queryTimestamp) +
      '_' + to_string(queryPoseRange) + ".ply";
  writer.write(mergedAlignedAccPlyFileName, *queryAccCloud, true);

  string queryPlyFileName =
      dataFolder + "/query_" + to_string(queryTimestamp) + ".ply";
  pcl::removeNaNFromPointCloud(*queryCloudRaw, *queryCloud, indices);
  writer.write(queryPlyFileName, *queryCloud, true);

  cout << "Finished to write to file:\n";
  cout << mergedAccumulatedPlyFileName << endl;
  cout << mergedAlignedPlyFileName << endl;
  cout << queryPlyFileName << endl;

  // visualization
  pcl::visualization::PCLVisualizer vis("vis");
  pcl::visualization::PointCloudColorHandlerCustom<Point> source_handler(
      mergedAccumulatedCloud, 0.0, 0.0, 255.0);
  pcl::visualization::PointCloudColorHandlerCustom<Point> aligned_handler(
      queryAccCloud, 0.0, 255.0, 0.0);
  pcl::visualization::PointCloudColorHandlerCustom<Point> target_handler(
      queryCloud, 255.0, 0.0, 0.0);
  vis.addPointCloud(mergedAccumulatedCloud, source_handler, "accumulated");
  vis.addPointCloud(queryAccCloud, aligned_handler, "alignedAcc");
  vis.addPointCloud(queryCloud, target_handler, "query");
  vis.spin();

  return 0;
}

void readLidarGpsExtrinsicParam(ifstream& file, Matrix4d& T_lidar2gps)
{
  string desc;

  Vector3d T;
  file >> desc >> T(0) >> T(1) >> T(2);

  double roll, pitch, yaw;
  file >> roll >> pitch >> yaw;
  Matrix3d R;
  R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

  T_lidar2gps.topLeftCorner(3, 3) = R;
  T_lidar2gps.topRightCorner(3, 1) = T;
}

void readLidarGpsPoses(ifstream& file, vector<uint64_t>& timestamps,
                       vector<Matrix4d>& lidar_gps_poses)
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

    timestamps.push_back(timestamp);
    lidar_gps_poses.push_back(curPose);
    // cout << "Pose #" << numPose << ":\n" << curPose << endl;
  }
}

bool findTimestampIndex(const uint64_t stamp,
                        const vector<uint64_t>& timestamps, size_t& index)
{
  auto iter =
      find_if(timestamps.begin(), timestamps.end(), [&stamp](const uint64_t& t)
              {
                return stamp == t;
              });

  if (iter != timestamps.end())
  {
    index = iter - timestamps.begin();
    return true;
  }
  else
    return false;
}

void cloudRangeFilter(const PointCloud::ConstPtr& inCloud, double minRange,
                      double maxRange, PointCloud::Ptr& outCloud)
{
  pcl::PointIndices::Ptr indices(new pcl::PointIndices());
  for (size_t i = 0; i < inCloud->points.size(); i++)
  {
    const Point& p = inCloud->points[i];
    double r = sqrt(p.x * p.x + p.y * p.y + p.z + p.z);

    if (minRange < r && r < maxRange)
      indices->indices.push_back(i);
  }

  pcl::ExtractIndices<Point> extract;
  extract.setInputCloud(inCloud);
  extract.setIndices(indices);
  extract.setNegative(false);
  extract.filter(*outCloud);
}
