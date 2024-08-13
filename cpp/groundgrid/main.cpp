#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/LU>
#include "GridMap.h"
#include "GroundGrid.h"
#include "GridMapConfig.h"


#include "test.h"


void Init();
void readSemanticKittiPc(const std::string& filename);
std::vector<uint32_t> readSemanticKittiLabel(const std::string& filename);
void processPoses(const std::string& poseFileName);
void sendPosition();
Eigen::Quaterniond quaternionFromMatrix(const Eigen::Matrix4d& matrix);
Eigen::Quaterniond quaternionFromeuler(double yaw, double pitch, double row); 
Eigen::Vector3d eulerFromQuaternion(const Eigen::Quaterniond& q);
Eigen::Quaterniond quaternionMultiply(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2);
void sendClouds();

std::vector<tPoint> pc;
std::vector<Eigen::Matrix4d> poses;
tPose odomPose;
int interestFrame = 20;

int main()
{
  // bigFind(2,3);
  // testA a;
  // a.testHello();
  // Eigen::Quaternionf q = quaternionFromeuler(0.7854, 0.1, 0);
  // std::cout << q.x() << "  " << q.y() << "  "<< q.z() << "  " << q.w() << std::endl;

  Init();
  GroundGrid ground_grid;
  ground_grid.update(odomPose);
}

void Init(){
  const std::string pcFileName = "/home/aiyang/SemanticKITTI/00/velodyne/000020.bin";
  const std::string poseFileName = "/home/aiyang/SemanticKITTI/00/poses.txt";
  readSemanticKittiPc(pcFileName);
  processPoses(poseFileName);
  sendPosition();
  sendClouds();
}

void readSemanticKittiPc(const std::string& filename){
  std::ifstream file(filename, std::ios::binary);

  while (file.peek()!=EOF)
  {
    tPoint point;
    float temX, temY, temZ, temIntensity;
    file.read(reinterpret_cast<char*>(&temX), sizeof(float));
    file.read(reinterpret_cast<char*>(&temY), sizeof(float));
    file.read(reinterpret_cast<char*>(&temZ), sizeof(float));
    file.read(reinterpret_cast<char*>(&temIntensity), sizeof(float));
    point.poseX =static_cast<double>(temX);
    point.poseY =static_cast<double>(temY);
    point.poseZ =static_cast<double>(temZ);
    point.intensity =static_cast<double>(temIntensity);
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
  std::string calibstring = "0.0004	-1.0000	-0.0081	-0.0120"
                            "-0.0072	0.0081	-0.9999	-0.0540"
                            "1.0000	0.0005	-0.0072	-0.2922";
  std::string invCalibstring = "0.0004	-0.0072	1.0000	0.2918"
                            "-1.0000	0.0081	0.0005	-0.0114"
                            "-0.0081	-0.9999	-0.0072	-0.0562";                  
  std::istringstream calibstream(calibstring);
  Eigen::Matrix<double, 3, 4> calib;
  for(int i=0; i<3;i++){
    for(int j=0;j<4;j++){
      calibstream>> calib(i,j);
    }
  }
  std::istringstream calibstream2(invCalibstring);
  Eigen::Matrix<double, 3, 4> inVcalib;
  for(int i=0; i<3;i++){
    for(int j=0;j<4;j++){
      calibstream2>> inVcalib(i,j);
    }
  }
  Eigen::Matrix4d calib_ext;
  calib_ext << calib, Eigen::RowVector4d(0,0,0,1);
  Eigen::Matrix4d calib_inv;
  calib_inv << inVcalib, Eigen::RowVector4d(0,0,0,1);

  std::ifstream file(poseFileName);
  std::string line;
  while (getline(file,line))
  {
    std::istringstream linestream(line);
    Eigen::Matrix<double,3,4> pose;
    
    for(int i=0;i<3;i++){
      for(int j=0;j<4;j++){
        linestream >> pose(i,j);    
      }
    }
    Eigen::Matrix4d pose_ext;
    pose_ext << pose, Eigen::RowVector4d(0,0,0,1);
    poses.push_back(calib_inv*(pose_ext*calib_ext));
  }
}

//odomPose stores the required transformation parameters pose(x,y,z) rotate(x,y,z,w) from kitti_base_link to map
//kitti_base_link to base_link 1.95 0 -1.73 0 0 0
//kitti_map to map -2.48 0 1.733 0 0 0 1
//map to odom 0 0 0 0 0 0 1
//kitti_base_link to velodyne 0 0 0 0 0 0
void sendPosition(){
  std::cout<< poses[interestFrame](0,3);
  odomPose.point.poseX = poses[interestFrame](0,3);
  odomPose.point.poseY = poses[interestFrame](1,3);
  odomPose.point.poseZ = poses[interestFrame](2,3);
  Eigen::Matrix4d R = poses[interestFrame];
  Eigen::Quaterniond q = quaternionFromMatrix(R);
  Eigen::Quaterniond q_rot = quaternionFromeuler(0,0,0); //No roatation here, so (0,0,0) input
  Eigen::Quaterniond q_new = quaternionMultiply(q_rot,q);
  odomPose.orientationX = q_new.x();
  odomPose.orientationY = q_new.y();
  odomPose.orientationZ = q_new.z();
  odomPose.orientationW = q_new.w();
}

Eigen::Quaterniond quaternionFromMatrix(const Eigen::Matrix4d& matrix){
  Eigen::Matrix3d roatationMatrix = matrix.block<3,3>(0,0);
  Eigen::Quaterniond quaternion(roatationMatrix);
  return quaternion;
}

Eigen::Quaterniond quaternionFromeuler(double yaw, double pitch, double roll){
  Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond quaternion = yaw_angle*pitch_angle*roll_angle;
  return quaternion;
}

Eigen::Vector3d eulerFromQuaternion(const Eigen::Quaterniond& q) {
    // Convert the quaternion to a 3x3 rotation matrix
    Eigen::Matrix3d rotationMatrix = q.toRotationMatrix();

    // Extract Euler angles from the rotation matrix (ZYX order)
    Eigen::Vector3d euler = rotationMatrix.eulerAngles(2, 1, 0);

    return euler; //order: yaw pitch roll (zyx)
}

Eigen::Quaterniond quaternionMultiply(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2){
  return q1*q2;
}


void sendClouds(){
  const std::string labelFileName = "/home/aiyang/SemanticKITTI/00/labels/000020.label";
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