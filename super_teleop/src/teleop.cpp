#include "teleop_robot.hpp"
#include "teleop_limb.hpp"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <razer_hydra/Hydra.h>
#include <atlas_msgs/AtlasState.h>
#include <atlas_msgs/AtlasCommand.h>
#include <pi_tracker/Skeleton.h>

#include <vector>
#include <map>
#include <boost/foreach.hpp>

tf::TransformBroadcaster br;
tf::TransformListener tl;
std::vector<TeleopLimb> limbs;

std::vector<std::string> ATLAS_JOINT_NAMES = {
  "back_lbz", "back_mby", "back_ubx", "neck_ay",
  "l_leg_uhz", "l_leg_mhx", "l_leg_lhy", "l_leg_kny", "l_leg_uay", "l_leg_lax",
  "r_leg_uhz", "r_leg_mhx", "r_leg_lhy", "r_leg_kny", "r_leg_uay", "r_leg_lax",
  "l_arm_usy", "l_arm_shx", "l_arm_ely", "l_arm_elx", "l_arm_uwy", "l_arm_mwx",  
  "r_arm_usy", "r_arm_shx", "r_arm_ely", "r_arm_elx", "r_arm_uwy", "r_arm_mwx" 
  };

void atlasCallback(const atlas_msgs::AtlasState::ConstPtr &msg)
{
  // Construct a mapping from limbs to positions
  std::map<std::string, double> joints;
  for (unsigned int i = 0; i < ATLAS_JOINT_NAMES.size(); ++i) {
    joints[ATLAS_JOINT_NAMES[i]] = msg->position[i];
  }

  // Update each limb with the new positions
  BOOST_FOREACH(TeleopLimb &limb, limbs) {
    limb.update(joints);
  }
}

void hydraCallback(const razer_hydra::Hydra::ConstPtr &msg)
{
  // Transform and scale hydra poses as necessary
  std::string paddle_tf[] = {"/left", "/right"};
  for (unsigned int i = 0; i < 2; ++i)
  {
    tf::Transform transform;
    tf::transformMsgToTF(msg->paddles[i].transform, transform);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), 
					  "/" + limbs[i].startLink(), paddle_tf[i]));
  }

  // Send command to left arm
  if (msg->paddles[0].trigger > 0.9) {
    
  }

  // Send command to right arm
  if (msg->paddles[1].trigger > 0.9) {

  }
}

void skeletonCallback(const pi_tracker::Skeleton::ConstPtr &msg)
{

}

int main(int argc, char* argv[])
{
  // Start up ROS node
  ros::init(argc, argv, "super_teleop");
  ros::NodeHandle nh;

  // Create a teleop robot
  TeleopRobot robot(nh);
  limbs.push_back(TeleopLimb(robot, "utorso", "l_hand"));
  limbs.push_back(TeleopLimb(robot, "utorso", "r_hand"));

  // Create a publisher for the commanded joints
  ros::Publisher pub = nh.advertise<atlas_msgs::AtlasCommand>("/atlas/atlas_command", 1);

  // Subscribe to all the necessary ROS topics
  nh.subscribe("/atlas/atlas_state", 1, atlasCallback);
  nh.subscribe("/arms/hydra_calib", 1, hydraCallback);
  nh.subscribe("/skeleton", 1, skeletonCallback);

  // Spin ROS node until shutdonw
  ros::spin();
}
