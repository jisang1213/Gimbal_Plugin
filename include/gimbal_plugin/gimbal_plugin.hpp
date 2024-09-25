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
#include <algorithm>

#include "raisin_plugin/plugin.hpp"
#include "raisin_parameter/parameter_container.hpp"
#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <sys/stat.h> // for mkdir

#include <unistd.h>
#include <string>
#include <filesystem>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "gimbal.hpp"
#include "recorder.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"  // Include the service definition

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
  // Callback for the service
  void handle_service(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                      std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  double start_time;
  std::ofstream ang_vel_log_base, ang_vel_log_cam, command_log, rpy_log, joint_state_log;
  RealSenseVideoRecorder* recorder_;

  parameter::ParameterContainer & param_;

  //Gimbal data types
  double dataToSend[3], receivedData[6];
  Gimbal gimbal;
  Eigen::Matrix3d rotL, rotR;
  Eigen::Vector3d posL, posR;
  bool receivedtopic = false;

  // Node and ROS 2 communication members
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr jointstate;
  rclcpp::Publisher<geometry_msgs::msg::Transform>::SharedPtr pubL, pubR;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;

  // Callback function for subscriber
  void messageCallback(const geometry_msgs::msg::Vector3::SharedPtr msg);
};

}  // namespace plugin

}  // namespace raisin

#endif  // GIMBAL_PLUGIN__GIMBAL_PLUGIN_HPP_
