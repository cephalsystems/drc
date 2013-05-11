#include "teleop_robot.hpp"
#include "teleop_limb.hpp"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <razer_hydra/Hydra.h>
#include <atlas_msgs/AtlasState.h>
#include <atlas_msgs/AtlasCommand.h>
#include <pi_tracker/Skeleton.h>

tf::TransformBroadcaster br;
tf::TransformListener tl;
std::vector<TeleopLimb> limbs;

void atlasCallback(const atlas_msgs::AtlasState::ConstPtr &msg)
{

}

void hydraCallback(const razer_hydra::Hydra::ConstPtr &msg)
{
  // Transform and scale hydra poses as necessary
  std::string paddle_tf[] = {"/left", "/right"};
  for (int i = 0; i < 2; ++i)
  {
    tf::Transform transform;
    tf::transformMsgToTF(msg->paddles[i].transform, transform);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), 
					  "/utorso", paddle_tf[i]));
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
