#include "teleop_robot.hpp"
#include "teleop_limb.hpp"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <razer_hydra/Hydra.h>
#include <atlas_msgs/AtlasState.h>
#include <atlas_msgs/AtlasCommand.h>
#include <sandia_hand_msgs/SimpleGraspSrv.h>
#include <pi_tracker/Skeleton.h>

#include <vector>
#include <map>
#include <boost/foreach.hpp>

tf::TransformBroadcaster *tf_out;
tf::TransformListener *tf_in;
std::vector<TeleopLimb> limbs;
std::map<std::string, double> commands;
std::map<std::string, double> joints;

ros::ServiceClient left_hand;
sandia_hand_msgs::SimpleGraspSrv left_hand_grasp;
ros::ServiceClient right_hand;
sandia_hand_msgs::SimpleGraspSrv right_hand_grasp;

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
  for (unsigned int i = 0; i < ATLAS_JOINT_NAMES.size(); ++i) {
    joints[ATLAS_JOINT_NAMES[i]] = msg->position[i];
  }
  
  // Update each limb with the new positions
  BOOST_FOREACH(TeleopLimb &limb, limbs) {
    limb.update(joints);
  }
}

void hydraArmsCallback(const razer_hydra::Hydra::ConstPtr &msg)
{
  std::string paddle_frame[] = {"/left_hand", "/right_hand"};
  tf::Transform paddle_tf[] = {
    tf::Transform(tf::Quaternion(0, 0, -1.5707)),
    tf::Transform(tf::Quaternion(0, 0,  1.5707)) 
  };

  // Transform and scale hydra poses as necessary
  for (unsigned int i = 0; i < 2; ++i) {

    // Extract transform from geometry message
    tf::Transform transform;
    tf::transformMsgToTF(msg->paddles[i].transform, transform);
    transform.setOrigin(transform.getOrigin() * 2.0);

    // Do a bit of an offset here
    tf::Transform offset;
    offset.setIdentity();
    offset.setOrigin(tf::Vector3(0.5, 0, -0.5));

    // Send out transform to TF for debugging
    transform = offset*transform*paddle_tf[i];
    tf_out->sendTransform(tf::StampedTransform(transform, ros::Time::now(), 
					       "/" + limbs[i].startLink(), paddle_frame[i]));
    
    // Add command to arm if trigger is pressed
    if (msg->paddles[i].trigger > 0.9) {
      std::map<std::string, double> cmds = limbs[i].solveEnd(transform);
      for ( std::map<std::string, double>::const_iterator it = cmds.begin();
	    it != cmds.end(); ++it ) {
	commands[it->first] = it->second;
      }
      ROS_INFO("Updating %s", paddle_frame[i].c_str());
    }
  }

  // Always set torso using joystick
  commands["back_lbz"] = joints["back_lbz"] + msg->paddles[0].joy[0] / 5.0;
  commands["back_mby"] = joints["back_mby"] + msg->paddles[0].joy[1] / 5.0;
  commands["back_ubx"] = joints["back_ubx"] + msg->paddles[1].joy[0] / 5.0;

  // TODO: make this not a cheap hack
  // Control the left hand
  if (msg->paddles[0].buttons[1]) {
    left_hand_grasp.request.grasp.name = "cylindrical";
    left_hand_grasp.request.grasp.closed_amount = 0.0;
    if (!left_hand.call(left_hand_grasp)) {
      ROS_ERROR("Failed to call hand service");
    }
  } else if (msg->paddles[0].buttons[3]) {
    left_hand_grasp.request.grasp.name = "cylindrical";
    left_hand_grasp.request.grasp.closed_amount = 1.0;
    if (!left_hand.call(left_hand_grasp)) {
      ROS_ERROR("Failed to call hand service");
    }
  }

  // Control the right hand
  if (msg->paddles[1].buttons[1]) {
    right_hand_grasp.request.grasp.name = "cylindrical";
    right_hand_grasp.request.grasp.closed_amount = 0.0;
    if (!right_hand.call(right_hand_grasp)) {
      ROS_ERROR("Failed to call hand service");
    }
  } else if (msg->paddles[1].buttons[3]) {
    right_hand_grasp.request.grasp.name = "cylindrical";
    right_hand_grasp.request.grasp.closed_amount = 1.0;
    if (!right_hand.call(right_hand_grasp)) {
      ROS_ERROR("Failed to call hand service");
    }
  }
}

void hydraLegsCallback(const razer_hydra::Hydra::ConstPtr &msg)
{
  std::string paddle_frame[] = {"/left_leg", "/right_leg"};
  tf::Transform paddle_tf[] = {
    tf::Transform(tf::Quaternion(0, 0, 0)),
    tf::Transform(tf::Quaternion(0, 0, 0)) 
  };

  // Transform and scale hydra poses as necessary
  for (unsigned int i = 0; i < 2; ++i) {

    // Extract transform from geometry message
    tf::Transform transform;
    tf::transformMsgToTF(msg->paddles[i].transform, transform);
    transform.setOrigin(transform.getOrigin() * 2.0);

    // Do a bit of an offset here
    tf::Transform offset;
    offset.setIdentity();
    offset.setOrigin(tf::Vector3(0.0, 0.0,-1.5));

    // Send out transform to TF for debugging
    transform = offset*transform*paddle_tf[i];
    tf_out->sendTransform(tf::StampedTransform(transform, ros::Time::now(), 
					       "/" + limbs[i+2].startLink(), paddle_frame[i]));
    
    // Add command to arm if trigger is pressed
    if (msg->paddles[i].trigger > 0.9) {
      std::map<std::string, double> cmds = limbs[i+2].solveEnd(transform);
      for ( std::map<std::string, double>::const_iterator it = cmds.begin();
	    it != cmds.end(); ++it ) {
	commands[it->first] = it->second;
      }
      ROS_INFO("Updating %s", paddle_frame[i].c_str());
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

  // Create TF objects
  tf_in = new tf::TransformListener();
  tf_out = new tf::TransformBroadcaster(); 

  // Create a teleop robot
  TeleopRobot robot(nh);
  limbs.push_back(TeleopLimb(robot, "utorso", "l_hand"));
  limbs.push_back(TeleopLimb(robot, "utorso", "r_hand"));
  limbs.push_back(TeleopLimb(robot, "pelvis", "l_foot"));
  limbs.push_back(TeleopLimb(robot, "pelvis", "r_foot"));

  // Create a publisher for the commanded joints
  ros::Publisher atlas_pub = nh.advertise<atlas_msgs::AtlasCommand>("/atlas/atlas_command", 1);

  // Subscribe to all the necessary ROS topics
  ros::Subscriber atlas_sub = nh.subscribe("/atlas/atlas_state", 1, atlasCallback);
  ros::Subscriber hydra_arms_sub = nh.subscribe("/arms/hydra_calib", 1, hydraArmsCallback);
  ros::Subscriber hydra_legs_sub = nh.subscribe("/legs/hydra_calib", 1, hydraLegsCallback);
  ros::Subscriber skel_sub = nh.subscribe("/skeleton", 1, skeletonCallback);

  // Subscribe to the hand services
  left_hand = nh.serviceClient<sandia_hand_msgs::SimpleGraspSrv>
    ("/sandia_hands/l_hand/simple_grasp");
  right_hand = nh.serviceClient<sandia_hand_msgs::SimpleGraspSrv>
    ("/sandia_hands/r_hand/simple_grasp");

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
  cmd_msg.k_effort.resize(n);
  cmd_msg.i_effort_min.resize(n);
  cmd_msg.i_effort_max.resize(n);

  // Initialize robot in some pose
  commands["l_arm_usy"] = -0.40;
  commands["l_arm_shx"] =  0.30;
  commands["l_arm_ely"] =  1.80;
  commands["l_arm_elx"] =  1.70;
  commands["l_arm_uwy"] = -0.90;
  commands["l_arm_mwx"] = -0.25;

  commands["r_arm_usy"] = -0.68;
  commands["r_arm_shx"] =  0.18;
  commands["r_arm_ely"] =  1.79; 
  commands["r_arm_elx"] = -1.75;
  commands["r_arm_uwy"] =  0.01;
  commands["r_arm_mwx"] = -0.44;

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
    cmd_msg.kd_position[i]  = kd * 10;
    cmd_msg.kp_velocity[i]  = 0;
    cmd_msg.k_effort[i]     = 0;

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
	cmd_msg.k_effort[i] = 255;
      } else {
	cmd_msg.k_effort[i] = 0;
      }
    }

    // Send out new joint command
    atlas_pub.publish(cmd_msg);

    // Wait for next cycle
    ros::spinOnce();
    r.sleep();
  }
}
