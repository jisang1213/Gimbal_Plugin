// Copyright (c) 2020 Robotics and Artificial Intelligence Lab, KAIST
//
// Any unauthorized copying, alteration, distribution, transmission,
// performance, display or use of this material is prohibited.
//
// All rights reserved.

#ifndef GIMBAL_PLUGIN__GIMBAL_PLUGIN_HPP_
#define GIMBAL_PLUGIN__GIMBAL_PLUGIN_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "raisin_plugin/plugin.hpp"
#include "raisin_parameter/parameter_container.hpp"
#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <fstream>

#include <unistd.h>
#include <string>
#include <filesystem>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "gimbal.hpp"
#include "recorder.hpp"


namespace raisin
{
namespace plugin
{

class Gimbal_Plugin : public Plugin
{
public:

  Gimbal_Plugin(
    raisim::World & world, raisim::RaisimServer & server,
    raisim::World & worldSim, raisim::RaisimServer & serverSim, GlobalResource & globalResource);

  ~Gimbal_Plugin();

  bool advance() final;

  bool init() final;

private:
  double start_time;
  std::ofstream ang_vel_log, command_log, rpy_log, joint_state_log;
  RealSenseVideoRecorder recorder_;

  parameter::ParameterContainer & param_;

  //Gimbal data types
  double dataToSend[3], receivedData[3];
  Gimbal gimbal;
  Eigen::Matrix3d rotL, rotR;
  Eigen::Vector3d posL, posR;
  bool receivedtopic = false;

  // Node and ROS 2 communication members
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr jointstate;
  rclcpp::Publisher<geometry_msgs::msg::Transform>::SharedPtr pubL, pubR;

  //lastest command message
  geometry_msgs::msg::Vector3 command_;

  // Callback function for subscriber
  void messageCallback(const geometry_msgs::msg::Vector3::SharedPtr msg);
};

}  // namespace plugin

}  // namespace raisin

#endif  // GIMBAL_PLUGIN__GIMBAL_PLUGIN_HPP_
