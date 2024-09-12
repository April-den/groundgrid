#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <dirent.h>
#include <Eigen/LU>
#include "GridMap.h"
#include "GroundGrid.h"
#include "GridMapConfig.h"
#include "GroundSegmentation.h"

using GroundGridPtr = std::shared_ptr<GroundGrid>;

void readPCDfile(const std::string finname, std::vector<tPoint> &points);
void readSemanticKittiPc(const std::string &filename);
std::vector<uint32_t> readSemanticKittiLabel(const std::string &filename);
void processPoses(const std::string &poseFileName);
void sendPosition();
Eigen::Quaterniond quaternionFromMatrix(const Eigen::Matrix4d &matrix);
Eigen::Quaterniond quaternionFromeuler(double yaw, double pitch, double row);
Eigen::Vector3d eulerFromQuaternion(const Eigen::Quaterniond &q);
Eigen::Quaterniond quaternionMultiply(const Eigen::Quaterniond &q1, const Eigen::Quaterniond &q2);
void sendClouds(const std::string labelFileName);
void points_callback(std::vector<tPoint> pc, groundgrid::GroundSegmentation &ground_segmentation_, const std::string segmentFileName);
void odom_callback(tPose inOdom);
// void publish_grid_map_layer(const std::string &layer_name, const int seq = 0);

std::vector<tPoint> pc;
std::vector<Eigen::Matrix4d> poses;
tPose odomPose;
int interestFrame = 20;
std::shared_ptr<grid_map::GridMap> map_ptr_;
GroundGridPtr groundgrid_;

int main()
{
  groundgrid_ = std::make_shared<GroundGrid>();
  groundgrid::GroundSegmentation ground_segmentation_;
  ground_segmentation_.init(groundgrid_->mDimension, groundgrid_->mResolution);

  std::string folderPath = "/home/aiyang/00/pcd";
  DIR *dir = opendir(folderPath.c_str());
  struct dirent *entry;
  std::vector<std::string> pcdFiles;
  int filesRead = 0;
  int maxFiles = 10; //the number of frames to read

  if (dir == nullptr)
  {
    std::cerr << "Could not open directory: " << folderPath << std::endl;
    return 1;
  }
  while ((entry = readdir(dir)) != nullptr)
  {
    std::string fileName = entry->d_name;

    // Skip "." and ".." entries
    if (fileName == "." || fileName == "..")
    {
      continue;
    }

    // Check if the file has the ".pcd" extension

    if (fileName.size() >= 4 && fileName.substr(fileName.size() - 4) == ".pcd")
    {
      pcdFiles.push_back(folderPath + "/" + fileName);
    }
  }
  closedir(dir);
  std::sort(pcdFiles.begin(), pcdFiles.end());

  for (const auto &filePath : pcdFiles)
  {
    if (filesRead < maxFiles)
    {
      std::cout << filePath << std::endl;
      size_t lastSlashPos = filePath.find_last_of("/");
      std::string fileName = filePath.substr(lastSlashPos + 1);
      size_t dotPos = fileName.find_last_of(".");
      std::string numberString = fileName.substr(0, dotPos);
      std::cout << numberString << std::endl;
      // Call the PCD reading function
      readPCDfile(filePath, pc);
      odom_callback(odomPose);
      points_callback(pc, ground_segmentation_, numberString);
      pc.clear();
      odomPose.reset();
      filesRead++;
    }
    else
    {
      break;
    }
  }
}

void readPCDfile(const std::string finname, std::vector<tPoint> &points)
{
  std::ifstream fin(finname, std::ios::binary);
  if (fin.bad())
  {
    std::cout << "Fail to open pcd file" << std::endl;
    return;
  }

  char s[11][1024]; // Header
  int Points_Num;
  std::string data_columns_type;
  std::string data_type;
  for (int i = 0; i < 11; ++i)
  {
    fin.getline(s[i], 1024);

    // FIELDS x y z rgb
    if (i == 2)
    {
      std::string s2 = s[2];
      size_t pos = s2.find("FIELDS");
      size_t size = s2.size();
      data_columns_type = s2.substr(pos + 7, size);
    }
    if (i == 8)
    {
      std::string s8 = s[8], poseString;
      size_t pos = s8.find("VIEWPOINT");
      size_t size = s8.size();
      poseString = s8.substr(pos + 10, size);
      std::istringstream iss(poseString);
      double value;
      iss >> odomPose.point.poseX >> odomPose.point.poseY >> odomPose.point.poseZ;
      iss >> odomPose.orientationW >> odomPose.orientationX >> odomPose.orientationY >> odomPose.orientationZ;
    }
    // POINTS xxx
    if (i == 9)
    {
      std::string s9 = s[9], Points_Str;
      size_t pos = s9.find("POINTS");
      size_t size = s9.size();
      Points_Str = s9.substr(pos + 7, size);
      Points_Num = atoi(Points_Str.c_str());
    }

    if (i == 10)
    {
      std::string s10 = s[10], DATA_SIZE;
      size_t pos = s10.find("DATA");
      size_t size = s10.size();
      data_type = s10.substr(pos + 5, size);
    }
  }

  tPoint p;
  if ((data_columns_type == "x y z intensity") && (data_type == "binary"))
  {
    std::cout << "start to read point ....." << std::endl;
    while (fin.peek() != EOF)
    {
      float temX, temY, temZ, temIntensity;
      fin.read(reinterpret_cast<char *>(&temX), sizeof(float));
      fin.read(reinterpret_cast<char *>(&temY), sizeof(float));
      fin.read(reinterpret_cast<char *>(&temZ), sizeof(float));
      fin.read(reinterpret_cast<char *>(&temIntensity), sizeof(float));
      p.poseX = static_cast<double>(temX) - odomPose.point.poseX;
      p.poseY = static_cast<double>(temY) - odomPose.point.poseY;
      p.poseZ = static_cast<double>(temZ) - odomPose.point.poseZ;
      p.intensity = static_cast<double>(temIntensity);
      points.push_back(p);
    }
  }
  else if ((data_columns_type == "x y z ") && (data_type == "binary"))
  {
    std::cout << "start to read point ....." << std::endl;
    while (fin.peek() != EOF)
    {
      float temX, temY, temZ, temIntensity;
      fin.read(reinterpret_cast<char *>(&temX), sizeof(float));
      fin.read(reinterpret_cast<char *>(&temY), sizeof(float));
      fin.read(reinterpret_cast<char *>(&temZ), sizeof(float));
      // fin.read(reinterpret_cast<char *>(&temIntensity), sizeof(float));
      p.poseX = static_cast<double>(temX) - odomPose.point.poseX;
      p.poseY = static_cast<double>(temY) - odomPose.point.poseY;
      p.poseZ = static_cast<double>(temZ) - odomPose.point.poseZ;
      // p.intensity = static_cast<double>(temIntensity);
      points.push_back(p);
    }
  }
  else
  {
    std::cout << "Point cloud format is not compatible" << std::endl;
  }
  fin.close();
}

void readSemanticKittiPc(const std::string &filename)
{
  std::ifstream file(filename, std::ios::binary);

  while (file.peek() != EOF)
  {
    tPoint point;
    float temX, temY, temZ, temIntensity;
    file.read(reinterpret_cast<char *>(&temX), sizeof(float));
    file.read(reinterpret_cast<char *>(&temY), sizeof(float));
    file.read(reinterpret_cast<char *>(&temZ), sizeof(float));
    file.read(reinterpret_cast<char *>(&temIntensity), sizeof(float));
    point.poseX = static_cast<double>(temX);
    point.poseY = static_cast<double>(temY);
    point.poseZ = static_cast<double>(temZ);
    point.intensity = static_cast<double>(temIntensity);
    pc.push_back(point);
  }
  file.close();
}

std::vector<uint32_t> readSemanticKittiLabel(const std::string &filename)
{
  std::vector<uint32_t> labels;
  std::ifstream file(filename, std::ios::binary);
  if (file.is_open())
  {
    uint32_t label;
    while (file.read(reinterpret_cast<char *>(&label), sizeof(uint32_t)))
    {
      labels.push_back(label);
    }
    file.close();
  }
  return labels;
}

void processPoses(const std::string &poseFileName)
{
  std::string calibstring = "0.0004	-1.0000	-0.0081	-0.0120"
                            "-0.0072	0.0081	-0.9999	-0.0540"
                            "1.0000	0.0005	-0.0072	-0.2922";
  std::string invCalibstring = "0.0004	-0.0072	1.0000	0.2918"
                               "-1.0000	0.0081	0.0005	-0.0114"
                               "-0.0081	-0.9999	-0.0072	-0.0562";
  std::istringstream calibstream(calibstring);
  Eigen::Matrix<double, 3, 4> calib;
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      calibstream >> calib(i, j);
    }
  }
  std::istringstream calibstream2(invCalibstring);
  Eigen::Matrix<double, 3, 4> inVcalib;
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      calibstream2 >> inVcalib(i, j);
    }
  }
  Eigen::Matrix4d calib_ext;
  calib_ext << calib, Eigen::RowVector4d(0, 0, 0, 1);
  Eigen::Matrix4d calib_inv;
  calib_inv << inVcalib, Eigen::RowVector4d(0, 0, 0, 1);

  std::ifstream file(poseFileName);
  std::string line;
  while (getline(file, line))
  {
    std::istringstream linestream(line);
    Eigen::Matrix<double, 3, 4> pose;

    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        linestream >> pose(i, j);
      }
    }
    Eigen::Matrix4d pose_ext;
    pose_ext << pose, Eigen::RowVector4d(0, 0, 0, 1);
    poses.push_back(calib_inv * (pose_ext * calib_ext));
  }
}

// odomPose stores the required transformation parameters pose(x,y,z) rotate(x,y,z,w) from kitti_base_link to map
// kitti_base_link to base_link 1.95 0 -1.73 0 0 0
// kitti_map to map -2.48 0 1.733 0 0 0 1
// map to odom 0 0 0 0 0 0 1
// kitti_base_link to velodyne 0 0 0 0 0 0
void sendPosition()
{
  odomPose.point.poseX = poses[interestFrame](0, 3);
  odomPose.point.poseY = poses[interestFrame](1, 3);
  odomPose.point.poseZ = poses[interestFrame](2, 3);
  Eigen::Matrix4d R = poses[interestFrame];
  Eigen::Quaterniond q = quaternionFromMatrix(R);
  Eigen::Quaterniond q_rot = quaternionFromeuler(0, 0, 0); // No roatation here, so (0,0,0) input
  Eigen::Quaterniond q_new = quaternionMultiply(q_rot, q);
  odomPose.orientationX = q_new.x();
  odomPose.orientationY = q_new.y();
  odomPose.orientationZ = q_new.z();
  odomPose.orientationW = q_new.w();
}

Eigen::Quaterniond quaternionFromMatrix(const Eigen::Matrix4d &matrix)
{
  Eigen::Matrix3d roatationMatrix = matrix.block<3, 3>(0, 0);
  Eigen::Quaterniond quaternion(roatationMatrix);
  return quaternion;
}

Eigen::Quaterniond quaternionFromeuler(double yaw, double pitch, double roll)
{
  Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond quaternion = yaw_angle * pitch_angle * roll_angle;
  return quaternion;
}

Eigen::Vector3d eulerFromQuaternion(const Eigen::Quaterniond &q)
{
  // Convert the quaternion to a 3x3 rotation matrix
  Eigen::Matrix3d rotationMatrix = q.toRotationMatrix();

  // Extract Euler angles from the rotation matrix (ZYX order)
  Eigen::Vector3d euler = rotationMatrix.eulerAngles(2, 1, 0);

  return euler; // order: yaw pitch roll (zyx)
}

Eigen::Quaterniond quaternionMultiply(const Eigen::Quaterniond &q1, const Eigen::Quaterniond &q2)
{
  return q1 * q2;
}

void sendClouds(const std::string labelFileName)
{

  std::vector<uint32_t> labels = readSemanticKittiLabel(labelFileName);
  if (labels.size() == pc.size())
  {
    for (int i = 0; i < pc.size(); i++)
    {
      pc[i].ring = labels[i];
    }
  }
  else
  {
    std::cout << "Label size doesn't equal point size" << std::endl;
    return;
  }
}

void odom_callback(tPose inOdom)
{
  static tPose lastOdom;
  // Delete "!lastOdom", skip the step of checking whether lastOdom is empty
  if (std::hypot(lastOdom.point.poseX - inOdom.point.poseX, 2.0f) + std::hypot(lastOdom.point.poseY - inOdom.point.poseY, 2.0f) >= 1.0)
  {
    map_ptr_ = groundgrid_->update(inOdom);
  }
}

void points_callback(std::vector<tPoint> pc, groundgrid::GroundSegmentation &ground_segmentation_, const std::string segmentFileName)
{
  static size_t time_vals = 0;
  tPose mapToBaseTransform, cloudOriginTransform;

  // Map not initialized yet, this means the node hasn't received any odom message so far.
  if (!map_ptr_)
    return;

  // origin code mapToBaseTransform = mTfBuffer.lookupTransform("map", "base_link", cloud_msg->header.stamp, ros::Duration(0.0));
  // function find transformation paramters from base_link to map
  mapToBaseTransform.point.poseX = mapToBaseTransform.point.poseX + odomPose.point.poseX;
  mapToBaseTransform.point.poseY = mapToBaseTransform.point.poseY + odomPose.point.poseY;
  mapToBaseTransform.point.poseZ = mapToBaseTransform.point.poseZ + odomPose.point.poseZ;

  mapToBaseTransform.point.poseX = mapToBaseTransform.point.poseX - kitti_base_to_baseX;
  mapToBaseTransform.point.poseY = mapToBaseTransform.point.poseY - kitti_base_to_baseY;
  mapToBaseTransform.point.poseZ = mapToBaseTransform.point.poseZ - kitti_base_to_baseZ;
  // origin code cloudOriginTransform = mTfBuffer.lookupTransform("map", "velodyne", cloud_msg->header.stamp, ros::Duration(0.0));
  // function find transformation paramters from velodyne to map,
  cloudOriginTransform.point.poseX = cloudOriginTransform.point.poseX + odomPose.point.poseX;
  cloudOriginTransform.point.poseY = cloudOriginTransform.point.poseY + odomPose.point.poseY;
  cloudOriginTransform.point.poseZ = cloudOriginTransform.point.poseZ + odomPose.point.poseZ;

  tPoint origin;

  origin.poseX = 0.0;
  origin.poseY = 0.0;
  origin.poseZ = 0.0;

  // tf2::doTransform(origin, origin, cloudOriginTransform);
  origin.poseX = origin.poseX + cloudOriginTransform.point.poseX;
  origin.poseY = origin.poseY + cloudOriginTransform.point.poseY;
  origin.poseZ = origin.poseZ + cloudOriginTransform.point.poseZ;

  // Transform cloud into map coordinate system
  // Transform to map
  std::vector<tPoint> transformed_cloud;
  tPose transformStamped;
  transformed_cloud.reserve(pc.size());
  transformStamped = cloudOriginTransform;
  tPoint psIn;

  for (int indx = 0; indx < pc.size(); indx++)
  {
    psIn = pc[indx];

    // tf2::doTransform(psIn, psIn, transformStamped);
    psIn.poseX = psIn.poseX + transformStamped.point.poseX;
    psIn.poseY = psIn.poseY + transformStamped.point.poseY;
    psIn.poseZ = psIn.poseZ + transformStamped.point.poseZ;

    transformed_cloud.push_back(psIn);
    pc[indx] = transformed_cloud[indx];
  }

  tPoint origin_pclPoint;
  origin_pclPoint = origin;
  std::cout << "Odom: " << origin_pclPoint.poseX << ", " << origin_pclPoint.poseY << ", " << origin_pclPoint.poseZ << std::endl;

  std::vector<tPoint> filteredCloud = ground_segmentation_.filter_cloud(pc, origin_pclPoint, mapToBaseTransform, *map_ptr_);

  // Write in txt file
  std::string segmFiName = "/home/aiyang/groundgrid/pcd/" + segmentFileName +".txt";
  std::ofstream file(segmFiName);
  // Check if the file opened successfully
  if (!file.is_open())
  {
    std::cerr << "Failed to open the file for writing." << std::endl;
  }
  file << std::fixed << std::setprecision(5);

  for (const auto &point : filteredCloud)
  {
    file << point.poseX << "\t" << point.poseY << "\t" << point.poseZ << "\t" << point.intensity << "\n";
  }
  file.close();
  std::cout << "Data written to file successfully.  " << filteredCloud.size() << std::endl;
}
