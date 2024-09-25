// Copyright (c) 2020 Robotics and Artificial Intelligence Lab, KAIST
//
// Any unauthorized copying, alteration, distribution, transmission,
// performance, display or use of this material is prohibited.
//
// All rights reserved.
// Created by You, Jisang 07/24

#include "gimbal_plugin/gimbal_plugin.hpp"
#include "gimbal_plugin/recorder.hpp"
#include <iostream>

namespace raisin
{
namespace plugin
{

Gimbal_Plugin::Gimbal_Plugin(
  raisim::World & world, raisim::RaisimServer & server,
  raisim::World & worldSim, raisim::RaisimServer & serverSim, GlobalResource & globalResource)
: Plugin(world, server, worldSim, serverSim, globalResource),
  param_(parameter::ParameterContainer::getRoot()["raisin_gimbal_com_plugin"])
{
  pluginType_ = PluginType::CUSTOM;

  //load the port name from the param file
  param_.loadFromPackageParameterFile("raisin_gimbal_com_plugin");

  //set serial port
  std::string portname = param_("gimbal_port");
  gimbal.setPortName(portname);

  // Get the home directory using getenv
  const char* homeDir = std::getenv("HOME");
  if (homeDir == nullptr) {
      std::cerr << "HOME environment variable not set" << std::endl;
  }

    // Generate timestamp for filename
  std::time_t now = std::time(nullptr);
  std::tm tm = *std::localtime(&now);
  std::stringstream ss;
  ss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
  std::string timestamp = ss.str();

  //open log files
  std::string log_dir = std::string(homeDir) + "/" + std::string(param_("log_directory"))  + "/" + timestamp;
  RSINFO("Logging to: " + log_dir);

  // Check if the directory exists
  struct stat info;
  if (stat(log_dir.c_str(), &info) != 0) {
      // Directory does not exist, create it
      std::cout << "Directory doesn't exist, creating it..." << std::endl;
      if (mkdir(log_dir.c_str(), 0777) != 0) {
          std::cerr << "Failed to create directory: " << log_dir << std::endl;
      }
  } else if (!(info.st_mode & S_IFDIR)) {
      std::cerr << log_dir << " is not a directory!" << std::endl;
  }

  recorder_ = new RealSenseVideoRecorder(log_dir);

  ang_vel_log_base.open(log_dir + "/" + "angular_vel_base.csv");
  ang_vel_log_cam.open(log_dir + "/" + "angular_vel_cam.csv");
  command_log.open(log_dir + "/" + "command.csv");
  rpy_log.open(log_dir + "/" + "rpy.csv");
  joint_state_log.open(log_dir + "/" + "joint_state.csv");

  // Check if the file was opened successfully
  if (!joint_state_log.is_open()) {
      RSINFO("Could not open CSV");
  }
  
  // Initialize the ROS 2 node and communication members
  node_ = rclcpp::Node::make_shared("gimbal_node");
  // Subscribe to the input topic
  sub = node_->create_subscription<geometry_msgs::msg::Vector3>(
          "gimbal_command", 10, std::bind(&Gimbal_Plugin::messageCallback, this, std::placeholders::_1));
  // Advertise the output topic
  jointstate = node_->create_publisher<geometry_msgs::msg::Vector3>("gimbal_joint_state", 10);
  pubL = node_->create_publisher<geometry_msgs::msg::Transform>("transforms/d435i_front_L", 10);
  pubR = node_->create_publisher<geometry_msgs::msg::Transform>("transforms/d435i_front_R", 10);
}

Gimbal_Plugin::~Gimbal_Plugin(){
  if (ang_vel_log_base.is_open()) {
      ang_vel_log_base.close();
  }
  if (ang_vel_log_cam.is_open()) {
      ang_vel_log_cam.close();
  }
  if (command_log.is_open()) {
      command_log.close();
  }
  if (rpy_log.is_open()) {
      rpy_log.close();
  }
  if (joint_state_log.is_open()) {
      joint_state_log.close();
  }
  delete recorder_;
}

bool Gimbal_Plugin::init()
{
  gimbal.configureGimbalPort();
  //recorder_.startRecording();
  start_time = worldHub_.getWorldTime();
  return true;
}

bool Gimbal_Plugin::advance()
{
  Eigen::Vector4d quat;
  Eigen::Vector3d angVel;
  Eigen::Matrix3d rot;
  Eigen::Vector3d eulerAng;
  double roll, pitch;
  double w,x,y,z;
  double timestamp;
  static int count = 0;   //for sinusoidal target for testing

  // read state
  robotHub_->lockMutex();
  // Eigen::VectorXd gv = robotHub_->getGeneralizedVelocity().e();
  auto imu = robotHub_->getSensorSet("base_imu")->getSensor<raisim::InertialMeasurementUnit>("imu");
  robotHub_->unlockMutex();

  quat = imu->getOrientation().e();
  angVel = imu->getAngularVelocity();

  //compute the roll and pitch from quaternion
  w = quat(0); x = quat(1); y = quat(2); z = quat(3);
  roll = std::atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y));
  double temp = 2*(w*y - z*x);

  if(std::abs(temp) >= 1)
    pitch = std::copysign(M_PI/2, temp);
  else
    pitch = std::asin(temp);

  //Workspace: -1.0 < x < 1.0, -0.5 < y < 1.2, -1.0 < z < 1.0

  //if the target was not set by topic callback, compute the default target
  if(!receivedtopic){
    //store target in the buffer to send
    dataToSend[0] = std::clamp(-roll, -1.0, 1.0);
    dataToSend[1] = std::clamp(-pitch, -0.5, 1.2);
    dataToSend[2] = std::clamp(-0.2*angVel[2], -1.0, 1.0); //0.9*sin(count/300.0);
  }
  

  //log data
  timestamp = worldHub_.getWorldTime() - start_time;
  rpy_log << timestamp << "," << roll << "," << pitch << "," << "\n";
  command_log << timestamp << "," << dataToSend[0] << "," << dataToSend[1]<< "," << dataToSend[2]<< "\n";
  ang_vel_log_base << timestamp << "," << angVel.x() << "," << angVel.y() << "," << angVel.z() << "\n";

  // quat = genCo_.segment<4>(3);
  // angVelW = genVel_.segment<3>(3);    //world frame

//TEST
    dataToSend[0] = timestamp;
    dataToSend[1] = timestamp;
    dataToSend[2] = timestamp;

  //write command to MCU
  while (write(gimbal.getPortID(), &dataToSend, sizeof(dataToSend)) < 0) {
      std::cerr << "Error writing to gimbal" << std::endl;
      gimbal.configureGimbalPort();
  }
  //reset receivedtarget to false after sending the command - race condition here if using multithreaded executor but probably ok
  receivedtopic = false;
  //get MCU response
  while (read(gimbal.getPortID(), &receivedData, sizeof(receivedData)) < 0) {
      std::cerr << "Error reading from gimbal" << std::endl;
      gimbal.configureGimbalPort();
  }

  auto state = geometry_msgs::msg::Vector3();
  state.x = receivedData[0];
  state.y = receivedData[1];
  state.z = receivedData[2];
  jointstate->publish(state);
  joint_state_log << timestamp << "," << state.x << "," << state.y << "," << state.z << "\n";
  //update the state in gimbal
  gimbal.setState(state.x, state.y, state.z);    //update the state in gimbal object

  // log gimbal angular velocity
  double gx = receivedData[3];
  double gy = receivedData[4];
  double gz = receivedData[5];
  ang_vel_log_cam << timestamp << "," << gx << "," << gy << "," << gz << "\n";

  rotL = gimbal.getRot_L();
  posL = gimbal.getPos_L();

  rotR = gimbal.getRot_R();
  posR = gimbal.getPos_R();

  // Convert the rotation matrix to a quaternion
  Eigen::Quaterniond quatL(rotL);
  Eigen::Quaterniond quatR(rotR);

  //Publish the transformation back to raisin
  auto transform_L = std::make_unique<geometry_msgs::msg::Transform>();
  auto transform_R = std::make_unique<geometry_msgs::msg::Transform>();

  //Left cam
  transform_L->translation.x = posL.x();
  transform_L->translation.y = posL.y();
  transform_L->translation.z = posL.z();
  
  transform_L->rotation.w = quatL.w();
  transform_L->rotation.x = quatL.x();
  transform_L->rotation.y = quatL.y();
  transform_L->rotation.z = quatL.z();
  
  //Right cam
  transform_R->translation.x = posR.x();
  transform_R->translation.y = posR.y();
  transform_R->translation.z = posR.z();
  
  transform_R->rotation.w = quatR.w();
  transform_R->rotation.x = quatR.x();
  transform_R->rotation.y = quatR.y();
  transform_R->rotation.z = quatR.z();

  // Publish the processed message to the output topic
  pubL->publish(std::move(transform_L));
  pubR->publish(std::move(transform_R));

  return true;
}

void Gimbal_Plugin::messageCallback(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
    // Store the latest command in buffer
    dataToSend[0] = msg->x;
    dataToSend[1] = msg->y;
    dataToSend[2] = msg->z;
    receivedtopic = true;
}

extern "C" Plugin * create(
  raisim::World & world, raisim::RaisimServer & server,
  raisim::World & worldSim, raisim::RaisimServer & serverSim, GlobalResource & globalResource)
{
  return new Gimbal_Plugin(world, server, worldSim, serverSim, globalResource);
}

extern "C" void destroy(Plugin * p)
{
  delete p;
}

}  // namespace plugin

}  // namespace raisin
