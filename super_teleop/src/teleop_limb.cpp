#include "teleop_limb.hpp"
#include <tf_conversions/tf_kdl.h>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include "chainiksolverpos_nr_jl_we.hpp"
#include <kdl/chainiksolverpos_lma.hpp>


TeleopLimb::TeleopLimb(TeleopRobot &robot, std::string start_link, std::string end_link) :
  start_link_(start_link), end_link_(end_link)
{
  // Find kinematic chain in tree
  KDL::Chain chain;
  if (!robot.kdl.getChain(start_link, end_link, chain)) {
    ROS_ERROR("Unable to find chain: %s -> %s", 
	      start_link.c_str(), end_link.c_str());
  }
  n_joints_ = chain.getNrOfJoints();
  q_current_ = KDL::JntArray(n_joints_);

  // Extract joint limits and names
  KDL::JntArray q_min(n_joints_);
  KDL::JntArray q_max(n_joints_);
  names_ = new std::string[n_joints_];
  for (int i = 0; i < n_joints_; ++i) {
    names_[i] = chain.getSegment(i).getJoint().getName();
    q_min.data[i] = robot.urdf.getJoint(names_[i])->limits->lower;
    q_max.data[i] = robot.urdf.getJoint(names_[i])->limits->upper;
  }

  // Initialize special IK velocity solver
  double weights[] = {1.0, 1.0, 1.0, 1e-5, 1e-5, 1e-5};

  KDL::ChainIkSolverVel_wdls *ikv = new KDL::ChainIkSolverVel_wdls(chain);
  ikv->setWeightTS(Eigen::DiagonalMatrix<double, 6, 6>(Eigen::Matrix<double, 6, 1>(weights)));

  // Initialize IK solvers
  fk_ = new KDL::ChainFkSolverPos_recursive(chain);
  ikv_ = ikv;
  
  KDL::ChainIkSolverPos_NR_JL_WE *ik = new KDL::ChainIkSolverPos_NR_JL_WE(chain, 
									  q_min, q_max, 
									  (*fk_), (*ikv_),
									  500, 1e-3);
  ik->setWeights(weights);
  ik_ = ik;
}

std::string TeleopLimb::startLink()
{
  return start_link_;
}

std::string TeleopLimb::endLink()
{
  return end_link_;
}

void TeleopLimb::update(std::map<std::string, double> pos)
{
  for (int i = 0; i < n_joints_; ++i) {
    q_current_.data[i] = pos[names_[i]];
  }
}

std::map<std::string, double> TeleopLimb::solveEnd(tf::Transform end)
{
  std::map<std::string, double> q_desired;

  // Convert from TF to KDL frame
  KDL::Frame frame;
  tf::TransformTFToKDL(end, frame);

  // Solve for closest IK solution
  KDL::JntArray q(n_joints_);
  int result = ik_->CartToJnt(q_current_, frame, q);
  if (result < 0) {
    ROS_ERROR("Bad IK solution. [%d]", result);
    return q_desired;
  }

  // Convert output to map from joint name to value
  for (int i = 0; i < n_joints_; ++i) {
    q_desired[names_[i]] = q.data[i];
  }
  return q_desired;
}

std::map<std::string, double> TeleopLimb::solveLimb(std::map<std::string, tf::Transform> limb)
{
  ROS_ERROR("Unsupported operation: solveLimb");
  return std::map<std::string, double>();
}



