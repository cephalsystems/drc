#ifndef __TELEOP_LIMB_HPP__
#define __TELEOP_LIMB_HPP__

#include "teleop_robot.hpp"

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainfksolver.hpp>
#include <urdf/model.h>

#include <string>
#include <map>

class TeleopLimb
{
public:
  TeleopLimb(TeleopRobot &robot, std::string start_link, std::string end_link);

  std::string startLink();
  std::string endLink();  

  void update(std::map<std::string, double> pos);

  std::map<std::string, double> solveEnd(tf::Transform end);
  std::map<std::string, double> solveLimb(std::map<std::string, tf::Transform> limb);

private:
  int n_joints_;
  std::string *names_;
  KDL::JntArray q_current_;

  std::string start_link_;
  std::string end_link_;

  KDL::ChainFkSolverPos *fk_;
  KDL::ChainIkSolverVel *ikv_;
  KDL::ChainIkSolverPos *ik_;
};

#endif /* __TELEOP_LIMB_HPP__ */
