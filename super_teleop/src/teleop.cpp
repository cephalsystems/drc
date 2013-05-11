#include "teleop_robot.hpp"
#include <ros/ros.h>

int main(int argc, char* argv[])
{
  // Start up ROS node
  ros::init(argc, argv, "super_teleop");
  ros::NodeHandle nh;

  // Create a teleop robot
  TeleopRobot robot(nh);

  // Spin ROS node until shutdonw
  ros::spin();
}
