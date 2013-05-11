#include "teleop_robot.hpp"
#include <ros/ros.h>

#include <razer_hydra/Hydra.h>
#include <atlas_msgs/AtlasState.h>
#include <atlas_msgs/AtlasCommand.h>
#include <pi_tracker/Skeleton.h>

void atlasCallback(const atlas_msgs::AtlasState::ConstPtr &msg)
{

}

void hydraCallback(const razer_hydra::Hydra::ConstPtr &msg)
{

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

  // Create a publisher for the commanded joints
  ros::Publisher pub = nh.advertise<atlas_msgs::AtlasCommand>("/atlas/atlas_command", 1);

  // Subscribe to all the necessary ROS topics
  nh.subscribe("/atlas/atlas_state", 1, atlasCallback);
  nh.subscribe("/arms/hydra_calib", 1, hydraCallback);
  nh.subscribe("/skeleton", 1, skeletonCallback);

  // Spin ROS node until shutdonw
  ros::spin();
}
