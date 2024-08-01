// Copyright (c) 2020 Robotics and Artificial Intelligence Lab, KAIST
//
// Any unauthorized copying, alteration, distribution, transmission,
// performance, display or use of this material is prohibited.
//
// All rights reserved.
// Created by You, Jisang 07/24

#include "gimbal_plugin/gimbal_plugin.hpp"
#include "gimbal_plugin/recorder.hpp"

namespace raisin
{
namespace plugin
{

Gimbal_Plugin::Gimbal_Plugin(
  raisim::World & world, raisim::RaisimServer & server,
  raisim::World & worldSim, raisim::RaisimServer & serverSim, GlobalResource & globalResource)
: Plugin(world, server, worldSim, serverSim, globalResource),
  param_(parameter::ParameterContainer::getRoot()["gimbal_plugin"])
{
  pluginType_ = PluginType::CUSTOM;

  //load the port name from the param file
  param_.loadFromPackageParameterFile("gimbal_plugin");

  //set serial port
  std::string portname = param_("gimbal_port");
  gimbal.setPortName(portname);

  //open log files
  std::string dir = param_("log_directory");
  ang_vel_log.open(dir + "/" + "angular_vel.csv");
  command_log.open(dir + "/" + "command.csv");
  rpy_log.open(dir + "/" + "rpy.csv");
  joint_state_log.open(dir + "/" + "joint_state.csv");
  
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
  if (ang_vel_log.is_open()) {
      ang_vel_log.close();
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
}

bool Gimbal_Plugin::init()
{
  gimbal.configureGimbalPort();
  recorder_.startRecording();
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

    //if the target was not set by topic callback, compute the default target
    if(!receivedtopic){
      //store target in the buffer to send
      dataToSend[0] = -roll;
      dataToSend[1] = -pitch-0.2;
      dataToSend[2] = -0.2*angVel[2];//0.9*sin(count/300.0);
    }

    //log data
    timestamp = worldHub_.getWorldTime() - start_time;
    rpy_log << timestamp << "," << roll << "," << pitch << "," << "/n";
    command_log << timestamp << "," << dataToSend[0] << "," << dataToSend[0]<< "," << dataToSend[0]<< "/n";
    ang_vel_log << timestamp << "," << angVel.x() << "," << angVel.y() << "," << angVel.z() << "/n";

    // quat = genCo_.segment<4>(3);
    // angVelW = genVel_.segment<3>(3);    //world frame

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
  
  timestamp = worldHub_.getWorldTime() - start_time;
  joint_state_log << timestamp << "," << state.x << "," << state.y << "," << state.z << "/n";

  //update the state in gimbal
  gimbal.setState(receivedData[0], receivedData[1], receivedData[2]);    //update the state in gimbal object

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
    // Store the latest message
    command_ = *msg;
    dataToSend[0] = command_.x;
    dataToSend[1] = command_.y;
    dataToSend[2] = command_.z;
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
