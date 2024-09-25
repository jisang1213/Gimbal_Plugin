//
// Created by You, Jisang on 06/19/2024.
//
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <stdexcept>
#include <iostream>
#include <cstring>
#include <cmath>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <thread>
#include <chrono>

//cooridinate frame class
class Frame{
  public:
    //rpy and xyz corresponds to URDF origin
    Eigen::Vector3d rpy;
    Eigen::Vector3d xyz;
    Eigen::Vector3d axis;
    double angle=0;
    bool isRev = false;
    Frame *parent = NULL; //pointer to parent frame

    Frame(){
      rpy.setZero();
      xyz.setZero();
    }

    Eigen::Vector3d getPos(){
      return xyz;
    }

    //this method returns the rotation matrix of the Frame/link
    Eigen::Matrix3d getRot(){
      double roll = rpy[0], pitch = rpy[1], yaw = rpy[2];
      Eigen::Matrix3d Rx, Ry, Rz, R;
      Rx << 1, 0, 0,
            0, cos(roll), -sin(roll),
            0, sin(roll), cos(roll);
      Ry << cos(pitch), 0, sin(pitch),
            0, 1, 0,
            -sin(pitch), 0, cos(pitch);
      Rz << cos(yaw), -sin(yaw), 0,
            sin(yaw), cos(yaw), 0,
            0, 0, 1;
      R = Rz*Ry*Rx;

      if(isRev){
        R = R * Eigen::AngleAxisd(angle, axis).toRotationMatrix();  //add joint rotation for revolute joint frames
      }
      return R;
    }
};

class Gimbal{
  public:
    void setState(double r, double p, double y){
      roll.angle = r;
      pitch.angle = p;
      yaw.angle = y;
    }

    Eigen::Matrix3d getRot_L(){
      return getRot(&Lcam);
    }
    Eigen::Vector3d getPos_L(){
      return getPos(&Lcam);
    }
    Eigen::Matrix3d getRot_R(){
      return getRot(&Rcam);
    }
    Eigen::Vector3d getPos_R(){
      return getPos(&Rcam);
    }

    void setPortName(std::string portname = "/dev/gimbal"){
      port = portname;
    }

    int getPortID(){
      return serial_fd;
    }

    Eigen::Vector3d getPos(Frame* frame){
      Eigen::Vector3d pos = Eigen::Vector3d::Zero();
      while(frame != NULL){
        pos = frame->getRot() * pos + frame->getPos();
        frame = frame->parent;
      }
      return pos;
    }

    Eigen::Matrix3d getRot(Frame* frame){
      Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
      while(frame != NULL){
        R = frame->getRot() * R;
        frame = frame->parent;
      }
      return R;
    }
    
    void configureGimbalPort();

    Gimbal(){
      //hard code gimbal URDF. Base is specified in the README. The origin is the robot IMU.
      base.xyz << 0.199, 0, 0.0935; //offset from robot imu
      base.rpy << 0,0,0;

      yaw.isRev = true;
      yaw.xyz << 0.12597, 0, 0.06489;
      yaw.rpy << 0, 0.78539816339, 0;
      yaw.axis << 0,0,1;
      yaw.parent = &base;

      roll.isRev = true;
      roll.xyz << -0.01866, 0, 0.04708;
      roll.rpy << 0, -0.523599, 0;
      roll.axis << 1,0,0;
      roll.parent = &yaw;

      pitch.isRev = true;
      pitch.xyz << 0.035, -0.004, 0;
      pitch.rpy << 0,0,0;
      pitch.axis << 0,1,0;
      pitch.parent = &roll;

      //CHANGE THESE TO CORRECT LENS LOCATION:
      Lcam_upright.xyz << 0.00832, 0.03985, 0.01096;
      Lcam_upright.rpy << 0, 0, 0.349066;
      Lcam_upright.parent = &pitch;

      Rcam_upright.xyz << 0.00832, -0.03385, 0.01096;
      Rcam_upright.rpy << 0, 0, -0.349066; 
      Rcam_upright.parent = &pitch;

      //Final rotation since camera is vertical
      Lcam.xyz << 0, 0, 0;
      Lcam.rpy << -M_PI/2, 0, 0;
      Lcam.parent = &Lcam_upright;

      Rcam.xyz << 0, 0, 0;
      Rcam.rpy << -M_PI/2, 0, 0; 
      Rcam.parent = &Rcam_upright;
    }

    ~Gimbal(){
      if (close(serial_fd) == -1) {
        std::cerr << "Failed to close gimbal serial port." << std::endl;
      }
    }
  
  private:
    Frame base, yaw, roll, pitch, Lcam_upright, Rcam_upright, Lcam, Rcam;
    std::string port;
    int serial_fd;
};

// Configure the serial port
void Gimbal::configureGimbalPort() {
    close(serial_fd);
    
    int baudrate = B115200;  // Baud rate
    serial_fd = -1;
    // Open serial port
    std::cerr<< "Opening port." << std::endl;
    while (serial_fd < 0) {
      serial_fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC); //  | O_NONBLOCK
    }
    std::cout << "Port opened." << std::endl;

    // Configure serial port settings
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(serial_fd, &tty) != 0) {
      std::cerr<< "Error getting gimbal serial port attributes." << std::endl;
    }

    cfsetospeed(&tty, baudrate);
    cfsetispeed(&tty, baudrate);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit characters
    tty.c_iflag &= ~IGNBRK;                         // Disable break processing
    tty.c_lflag = 0;                                // No signaling characters, no echo, no canonical processing
    tty.c_oflag = 0;                                // No remapping, no delays
    tty.c_cc[VMIN]  = 0;                            // Read doesn't block
    tty.c_cc[VTIME] = 1;                            // 0.1 seconds read timeout
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // Shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD);                // Ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);              // No parity
    tty.c_cflag &= ~CSTOPB;                         // 1 stop bit
    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
      std::cerr<< "Error setting gimbal serial port attributes."<< std::endl;
      //RSFATAL("Error setting gimbal serial port attributes");
    }
}
