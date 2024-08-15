#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/LU>
#include "GridMap.h"
#include "GroundGrid.h"
#include "GridMapConfig.h"
#include "GroundSegmentation.h"

using GroundGridPtr = std::shared_ptr<GroundGrid>;

void Init();
void readSemanticKittiPc(const std::string &filename);
std::vector<uint32_t> readSemanticKittiLabel(const std::string &filename);
void processPoses(const std::string &poseFileName);
void sendPosition();
Eigen::Quaterniond quaternionFromMatrix(const Eigen::Matrix4d &matrix);
Eigen::Quaterniond quaternionFromeuler(double yaw, double pitch, double row);
Eigen::Vector3d eulerFromQuaternion(const Eigen::Quaterniond &q);
Eigen::Quaterniond quaternionMultiply(const Eigen::Quaterniond &q1, const Eigen::Quaterniond &q2);
void sendClouds();
void points_callback(std::vector<tPoint> pc);
void odom_callback(tPose inOdom);
void publish_grid_map_layer(const std::string &layer_name, const int seq = 0);

std::vector<tPoint> pc;
std::vector<Eigen::Matrix4d> poses;
tPose odomPose;
int interestFrame = 20;
std::shared_ptr<grid_map::GridMap> map_ptr_;
GroundGridPtr groundgrid_;

int main()
{
  Init();

  groundgrid_ = std::make_shared<GroundGrid>();
  odom_callback(odomPose);
  points_callback(pc);
}

void Init()
{
  const std::string pcFileName = "/home/aiyang/SemanticKITTI/00/velodyne/000020.bin";
  const std::string poseFileName = "/home/aiyang/SemanticKITTI/00/poses.txt";
  readSemanticKittiPc(pcFileName);
  processPoses(poseFileName);
  sendPosition();
  sendClouds();
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
  std::cout << poses[interestFrame](0, 3);
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

void sendClouds()
{
  const std::string labelFileName = "/home/aiyang/SemanticKITTI/00/labels/000020.label";
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

void points_callback(std::vector<tPoint> pc)
{

  static size_t time_vals = 0;
  tPose mapToBaseTransform, cloudOriginTransform;

  // Map not initialized yet, this means the node hasn't received any odom message so far.
  if (!map_ptr_)
    return;

  // origin code mapToBaseTransform = mTfBuffer.lookupTransform("map", "base_link", cloud_msg->header.stamp, ros::Duration(0.0));
  // function find transformation paramters from base_link to map
  mapToBaseTransform.point.poseX = mapToBaseTransform.point.poseX + odomPose.point.poseX;
  mapToBaseTransform.point.poseY = odomPose.point.poseY + mapToBaseTransform.point.poseY;
  mapToBaseTransform.point.poseZ = odomPose.point.poseZ + mapToBaseTransform.point.poseZ;

  mapToBaseTransform.point.poseX = mapToBaseTransform.point.poseX + kitti_base_to_baseX;
  mapToBaseTransform.point.poseY = mapToBaseTransform.point.poseY + kitti_base_to_baseY;
  mapToBaseTransform.point.poseZ = mapToBaseTransform.point.poseZ + kitti_base_to_baseZ;
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
    psIn.poseX = pc[indx].poseX;
    psIn.poseY = pc[indx].poseY;
    psIn.poseZ = pc[indx].poseZ;

    // tf2::doTransform(psIn, psIn, transformStamped);
    psIn.poseX = psIn.poseX + transformStamped.point.poseX;
    psIn.poseY = psIn.poseY + transformStamped.point.poseY;
    psIn.poseZ = psIn.poseZ + transformStamped.point.poseZ;

    transformed_cloud[indx] = psIn;
    pc[indx] = transformed_cloud[indx];
  }

  auto end = std::chrono::steady_clock::now();
  ROS_DEBUG_STREAM("cloud transformation took " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms");

  auto start2 = std::chrono::steady_clock::now();
  std::clock_t c_clock = std::clock();
  sensor_msgs::PointCloud2 cloud_msg_out;
  PCLPoint origin_pclPoint;
  origin_pclPoint.x = origin.point.x;
  origin_pclPoint.y = origin.point.y;
  origin_pclPoint.z = origin.point.z;
  pcl::toROSMsg(*(ground_segmentation_.filter_cloud(cloud, origin_pclPoint, mapToBaseTransform, *map_ptr_)), cloud_msg_out);

  cloud_msg_out.header = cloud_msg->header;
  cloud_msg_out.header.frame_id = "map";
  filtered_cloud_pub_.publish(cloud_msg_out);
  end = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start2;
  const double milliseconds = elapsed_seconds.count() * 1000;
  const double c_millis = double(std::clock() - c_clock) / CLOCKS_PER_SEC * 1000;
  avg_time = (milliseconds + time_vals * avg_time) / (time_vals + 1);
  avg_cpu_time = (c_millis + time_vals * avg_cpu_time) / (time_vals + 1);
  ++time_vals;
  ROS_INFO_STREAM("groundgrid took " << milliseconds << "ms (avg: " << avg_time << "ms)");
  ROS_DEBUG_STREAM("total cpu time used: " << c_millis << "ms (avg: " << avg_cpu_time << "ms)");

  grid_map_msgs::GridMap grid_map_msg;
  grid_map::GridMapRosConverter::toMessage(*map_ptr_, grid_map_msg);
  grid_map_msg.info.header.stamp = cloud_msg->header.stamp;
  grid_map_pub_.publish(grid_map_msg);

  const ros::NodeHandle &nh = getNodeHandle();
  image_transport::ImageTransport it(nh);

  for (const auto &layer : map_ptr_->getLayers())
  {
    if (layer_pubs_.find(layer) == layer_pubs_.end())
    {
      layer_pubs_[layer] = it.advertise("/groundgrid/grid_map_cv_" + layer, 1);
    }
    publish_grid_map_layer(layer_pubs_.at(layer), layer, cloud_msg->header.seq, cloud_msg->header.stamp);
  }

  if (terrain_im_pub_.getNumSubscribers())
  {
    publish_grid_map_layer(terrain_im_pub_, "terrain", cloud_msg->header.seq, cloud_msg->header.stamp);
  }

  end = std::chrono::steady_clock::now();
  ROS_DEBUG_STREAM("overall " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms");
}

void publish_grid_map_layer(const std::string &layer_name, const int seq = 0)
{
  cv::Mat img, normalized_img, color_img, mask;

  if (pub.getNumSubscribers())
  {
    if (layer_name != "terrain")
    {
      const auto &map = *map_ptr_;
      grid_map::GridMapCvConverter::toImage<unsigned char, 1>(map, layer_name, CV_8UC1, img);
      cv::applyColorMap(img, color_img, cv::COLORMAP_TWILIGHT);

      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "8UC3", color_img).toImageMsg();
      msg->header.stamp = stamp;
      pub.publish(msg);
    }
    else
    { // special treatment for the terrain evaluation
      const auto &map = *map_ptr_;
      img = cv::Mat(map.getSize()(0), map.getSize()(1), CV_32FC3, cv::Scalar(0, 0, 0));
      normalized_img = cv::Mat(map.getSize()(0), map.getSize()(1), CV_32FC3, cv::Scalar(0, 0, 0));
      const grid_map::Matrix &data = map["ground"];
      const grid_map::Matrix &visited_layer = map["pointsRaw"];
      const grid_map::Matrix &gp_layer = map["groundpatch"];
      const float &car_height = data(181, 181);
      const float &ground_min = map["ground"].minCoeff() - car_height;
      const float &ground_max = map["ground"].maxCoeff() - car_height;
      if (ground_max == ground_min)
        return;

      for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator)
      {
        const grid_map::Index index(*iterator);
        const float &value = data(index(0), index(1)); // - car_height;
        const float &gp = gp_layer(index(0), index(1));
        const grid_map::Index imageIndex(iterator.getUnwrappedIndex());
        const float &pointssum = visited_layer.block<3, 3>(index(0) - 1, index(1) - 1).sum();
        const float &pointcount = visited_layer(index(0), index(1));

        // img.at<cv::Point3f>(imageIndex(0), imageIndex(1)) = cv::Point3f(value, / *pointcount+1 * /pointcount >= 3 ? 1.0f : 0.0f, gp > .0 ? gp : 0.0f);
        img.at<cv::Point3f>(imageIndex(0), imageIndex(1)) = cv::Point3f(value, pointssum >= 27 ? 1.0f : 0.0f, pointcount);
      }

      geometry_msgs::TransformStamped baseToUtmTransform;

      try
      {
        baseToUtmTransform = mTfBuffer.lookupTransform("utm", "base_link", stamp, ros::Duration(0.0));
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN("%s", ex.what());
        return;
      }
      geometry_msgs::PointStamped ps;
      ps.header.frame_id = "base_link";
      ps.header.stamp = stamp;
      tf2::doTransform(ps, ps, baseToUtmTransform);

      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "32FC3", img).toImageMsg();
      msg->header.frame_id = std::to_string(seq) + "_" + std::to_string(ps.point.x) + "_" + std::to_string(ps.point.y);
      pub.publish(msg);
    }
  }
}