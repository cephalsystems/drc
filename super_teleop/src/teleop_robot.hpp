#ifndef __TELEOP_ROBOT_HPP__
#define __TELEOP_ROBOT_HPP__

#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

class TeleopRobot
{
public:
  TeleopRobot(ros::NodeHandle &nh);

  urdf::Model urdf;
  KDL::Tree kdl;
};

#endif __TELEOP_ROBOT_HPP__
