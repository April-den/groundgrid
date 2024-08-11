#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
// #include "GridMap.h"
#include "GroundGrid.h"

#include "test.h"


void Init();
void readSemanticKittiPc(const std::string& filename);
std::vector<uint32_t> readSemanticKittiLabel(const std::string& filename);
void processPoses(const std::string& poseFileName);
void sendPosition();
Eigen::Quaternionf quaternionFromMatrix(const Eigen::Matrix4f& matrix);
Eigen::Quaternionf quaternionFromeuler(float yaw, float pitch, float row); 
Eigen::Vector3f eulerFromQuaternion(const Eigen::Quaternionf& q);
Eigen::Quaternionf quaternionMultiply(const Eigen::Quaternionf& q1, const Eigen::Quaternionf& q2);
void sendClouds();

std::vector<tPoint> pc;
std::vector<Eigen::Matrix4f> poses;
tPose odomPose;
int interestFrame = 0;

// #include <pcl/point_types.h>
int main()
{
  // bigFind(2,3);
  // testA a;
  // a.testHello();
  // Eigen::Quaternionf q = quaternionFromeuler(0.7854, 0.1, 0);
  // std::cout << q.x() << "  " << q.y() << "  "<< q.z() << "  " << q.w() << std::endl;

  Init();
  GroundGrid ground_grid;
  ground_grid.initGroundGrid(odomPose);
  ground_grid.update(odomPose);
}

void Init(){
  const std::string pcFileName = "/home/aiyang/SemanticKITTI/00/velodyne/000000.bin";
  const std::string poseFileName = "/home/aiyang/SemanticKITTI/00/poses.txt";
  readSemanticKittiPc(pcFileName);
  processPoses(poseFileName);
  std::cout << poses[interestFrame](0,3) << "  "<< poses[interestFrame](1,3) << "  " << poses[interestFrame](2,3) << std::endl;
  sendPosition();
  std::cout << odomPose.point.poseX<< "  "<< odomPose.point.poseY << "  " << odomPose.point.poseZ  <<  "  "
            << odomPose.orientationX << "  " << odomPose.orientationY << "  " << odomPose.orientationZ << odomPose.orientationW<< std::endl;
  sendClouds();
}

void readSemanticKittiPc(const std::string& filename){
  std::ifstream file(filename, std::ios::binary);

  while (file.peek()!=EOF)
  {
    tPoint point;
    file.read(reinterpret_cast<char*>(&point.poseX), sizeof(float));
    file.read(reinterpret_cast<char*>(&point.poseY), sizeof(float));
    file.read(reinterpret_cast<char*>(&point.poseZ), sizeof(float));
    file.read(reinterpret_cast<char*>(&point.intensity), sizeof(float));
    pc.push_back(point);
  }
  file.close();
}

std::vector<uint32_t> readSemanticKittiLabel(const std::string& filename){
  std::vector<uint32_t> labels;
  std::ifstream file(filename, std::ios::binary);
  if(file.is_open()){
    uint32_t label;
    while(file.read(reinterpret_cast<char*>(&label), sizeof(uint32_t))){
      labels.push_back(label);
    }
    file.close();
  }
  return labels;
}

void processPoses(const std::string& poseFileName){
  std::string calibstring = "4.276802385584e-04 -9.999672484946e-01 -8.084491683471e-03 -1.198459927713e-02"
                            "-7.210626507497e-03 8.081198471645e-03 -9.999413164504e-01 -5.403984729748e-02"
                            "9.999738645903e-01 4.859485810390e-04 -7.206933692422e-03 -2.921968648686e-01";
  std::istringstream calibstream(calibstring);
  Eigen::Matrix<float, 3, 4> calib;
  for(int i=0; i<3;i++){
    for(int j=0;j<4;j++){
      calibstream>> calib(i,j);
    }
  }
  Eigen::Matrix4f calib_ext;
  calib_ext << calib, Eigen::RowVector4f(0,0,0,1);
  Eigen::Matrix4f calib_inv = calib_ext.inverse();
  std::ifstream file(poseFileName);
  std::string line;
  while (getline(file,line))
  {
    std::istringstream linestream(line);
    Eigen::Matrix<float,3,4> pose;
    for(int i=0;i<3;i++){
      for(int j=0;j<3;j++){
        linestream >> pose(i,j);
      }
    }
    Eigen::Matrix4f pose_ext;
    pose_ext << pose, Eigen::RowVector4f(0,0,0,1);
    poses.push_back(calib_inv*(pose_ext*calib_ext));
  }
}
//odomPose stores the required transformation parameters pose(x,y,z) rotate(x,y,z,w) from kitti_base_link to map
//kitti_base_link to base_link 1.95 0 -1.73 0 0 0
//kitti_map to map -2.48 0 1.733 0 0 0 1
//map to odom 0 0 0 0 0 0 1
//kitti_base_link to velodyne 0 0 0 0 0 0
void sendPosition(){
  odomPose.point.poseX = poses[interestFrame](0,3);
  odomPose.point.poseY = poses[interestFrame](1,3);
  odomPose.point.poseZ = poses[interestFrame](2,3);
  Eigen::Matrix4f R = poses[interestFrame];
  Eigen::Quaternionf q = quaternionFromMatrix(R);
  Eigen::Quaternionf q_rot = quaternionFromeuler(0,0,0); //No roatation here, so (0,0,0) input
  Eigen::Quaternionf q_new = quaternionMultiply(q_rot,q);
  odomPose.orientationX = q_new.x();
  odomPose.orientationY = q_new.y();
  odomPose.orientationZ = q_new.z();
  odomPose.orientationW = q_new.w();
}

Eigen::Quaternionf quaternionFromMatrix(const Eigen::Matrix4f& matrix){
  Eigen::Matrix3f roatationMatrix = matrix.block<3,3>(0,0);
  Eigen::Quaternionf quaternion(roatationMatrix);
  return quaternion;
}

Eigen::Quaternionf quaternionFromeuler(float yaw, float pitch, float roll){
  Eigen::AngleAxisf roll_angle(roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf pitch_angle(pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf yaw_angle(yaw, Eigen::Vector3f::UnitZ());
  Eigen::Quaternionf quaternion = yaw_angle*pitch_angle*roll_angle;
  return quaternion;
}

Eigen::Vector3f eulerFromQuaternion(const Eigen::Quaternionf& q) {
    // Convert the quaternion to a 3x3 rotation matrix
    Eigen::Matrix3f rotationMatrix = q.toRotationMatrix();

    // Extract Euler angles from the rotation matrix (ZYX order)
    Eigen::Vector3f euler = rotationMatrix.eulerAngles(2, 1, 0);

    return euler; //order: yaw pitch roll (zyx)
}

Eigen::Quaternionf quaternionMultiply(const Eigen::Quaternionf& q1, const Eigen::Quaternionf& q2){
  return q1*q2;
}

void sendClouds(){
  const std::string labelFileName = "/home/aiyang/SemanticKITTI/00/labels/000000.label";
  std::vector<uint32_t> labels = readSemanticKittiLabel(labelFileName);
  if(labels.size()==pc.size()){
    for(int i=0; i<pc.size();i++){
      pc[i].ring = labels[i];
    }
  }else{
    std::cout<<"Label size doesn't equal point size"<<std::endl;
    return;
  }
}