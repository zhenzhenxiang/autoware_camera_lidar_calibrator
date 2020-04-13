#include <iostream>
#include <fstream>
#include <sstream>

#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

#include <opencv2/opencv.hpp>

typedef pcl::PointXYZI Point;
typedef pcl::PointCloud<Point> PointCloud;

using namespace std;
using namespace ros;
using namespace cv;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "convert_from_txt_to_ply");
  ros::NodeHandle nh;

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  if (args.size() < 2)
  {
    ROS_INFO("Usage: rosrun autoware_camera_lidar_calibrator "
             "convert_from_txt_to_ply data_folder");
    return 1;
  }

  // input files
  string dataFolder = args[1];
  string dataFilePath = dataFolder + "/data.txt";
  string outputListFilePath = dataFolder + "/data_ply.txt";

  // get file list
  vector<string> fileList;
  ifstream dataFile(dataFilePath.c_str());

  if (!dataFile.is_open())
  {
    cerr << "Can not open txt file: " << dataFilePath << endl;
    return 1;
  }

  string fileName;
  while (dataFile >> fileName)
  {
    fileList.push_back(fileName);
  }

  cout << fileList.size() << " files loaded!" << endl;

  // convert and save to PLY file
  for (auto f : fileList)
  {
    // -- open data file
    ifstream inputFile(f.c_str());
    if (!inputFile.is_open())
    {
      cerr << "Can not open data file: " << f << endl;
      return 1;
    }

    // -- read cloud data
    string line;
    int lineNum = 0;
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::PLYWriter writer;
    ofstream outputListFile(outputListFilePath, ios_base::app);

    while (getline(inputFile, line))
    {
      lineNum++;

      stringstream linestream(line);
      string valueStr;
      vector<double> values;

      while (getline(linestream, valueStr, ','))
      {
        values.push_back(atof(valueStr.c_str()));
      }

      if (values.size() != 4)
      {
        cout << "Warning: Invalid data in line #" << lineNum << ", file: " << f
             << endl;
        continue;
      }

      // -- ignore NaN
      if (values[3] < 1e-6)
        continue;

      pcl::PointXYZI pt;
      pt.x = values[0];
      pt.y = values[1];
      pt.z = values[2];
      pt.intensity = values[3];

      cloud.push_back(pt);
    }

    cout << cloud.points.size() << " points loaded from file " << f << endl;

    if (cloud.points.size() == 0)
      continue;

    // -- write to ply file
    string plyFile = f;
    plyFile.replace(plyFile.size() - 4, plyFile.size(), ".ply");
    writer.write(plyFile, cloud, true);

    // -- write ply list file
    outputListFile << plyFile << endl;
  }

  cout << "Finished!" << endl;

  return 0;
}
