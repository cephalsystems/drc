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
std::map<std::string, double> commands;

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

    // Add command to arm if trigger is pressed
    if (msg->paddles[i].trigger > 0.9) {
      std::map<std::string, double> cmds = limbs[i].solveEnd(transform);
      commands.insert(cmds.begin(), cmds.end());
    }
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

  // Create an ATLAS command
  unsigned int n = ATLAS_JOINT_NAMES.size();
  atlas_msgs::AtlasCommand cmd_msg;
  cmd_msg.position.resize(n);
  cmd_msg.velocity.resize(n);
  cmd_msg.effort.resize(n);
  cmd_msg.kp_position.resize(n);
  cmd_msg.ki_position.resize(n);
  cmd_msg.kd_position.resize(n);
  cmd_msg.kp_velocity.resize(n);
  cmd_msg.i_effort_min.resize(n);
  cmd_msg.i_effort_max.resize(n);

  // Fill in PID values
  std::string gains_prefix = "atlas_controller/gains/";
  for (unsigned int i = 0; i < n; ++i) {
    double kp, ki, kd, i_clamp;

    nh.getParam("atlas_controller/gains/" + ATLAS_JOINT_NAMES[i] + "/p", kp);
    nh.getParam("atlas_controller/gains/" + ATLAS_JOINT_NAMES[i] + "/i", ki);
    nh.getParam("atlas_controller/gains/" + ATLAS_JOINT_NAMES[i] + "/d", kd);
    nh.getParam("atlas_controller/gains/" + ATLAS_JOINT_NAMES[i] + "/i_clamp", i_clamp);

    cmd_msg.kp_position[i]  = kp;
    cmd_msg.ki_position[i]  = ki;
    cmd_msg.kd_position[i]  = kd;
    cmd_msg.kp_velocity[i]  = 0;

    cmd_msg.i_effort_min[i] = -i_clamp;
    cmd_msg.i_effort_max[i] =  i_clamp;
    
    cmd_msg.position[i]     = 0;
    cmd_msg.velocity[i]     = 0;
    cmd_msg.effort[i]       = 0;
  }

  // Send ATLAS commands at a fixed rate
  ros::Rate r(100);
  while(ros::ok()) {
    
    // Fill in command from current joint table
    for (unsigned int i = 0; i < ATLAS_JOINT_NAMES.size(); ++i) {

      // Look for a command for this joint
      std::map<std::string, double>::iterator it = 
	commands.find(ATLAS_JOINT_NAMES[i]);
      
      // If it exists, fill it in
      if (it != commands.end()) {
	cmd_msg.position[i] = it->second;
	cmd_msg.effort[i] = 255;
      } else {
	cmd_msg.effort[i] = 0;
      }
    }

    // Wait for next cycle
    ros::spinOnce();
    r.sleep();
  }
}
