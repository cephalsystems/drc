#ifndef __TELEOP_ROBOT_HPP__
#define __TELEOP_ROBOT_HPP__

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainfksolver.hpp>
#include <urdf/model.h>

#include <string>
#include <map>

class TeleopRobot
{
public:
  TeleopRobot(ros::NodeHandle &nh);

  urdf::Model urdf;
  KDL::Tree kdl;
};

class TeleopLimb
{
public:
  TeleopLimb(TeleopRobot &robot, std::string start_link, std::string end_link);

  void update(std::map<std::string, double> pos);

  std::map<std::string, double> solveEnd(tf::Transform end);
  std::map<std::string, double> solveLimb(std::map<std::string, tf::Transform> limb);
  std::map<std::string, double> clear();

private:
  int n_joints_;
  std::map<std::string, int> names_to_idx_;
  std::vector<std::string> idx_to_names_;
  KDL::JntArray q_current_;

  KDL::ChainFkSolverPos *fk_;
  KDL::ChainIkSolverVel *ikv_;
  KDL::ChainIkSolverPos *ik_;
};

#endif /* __TELEOP_ROBOT_HPP__ */
