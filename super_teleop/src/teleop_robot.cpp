#include "teleop_robot.hpp"
#include <string>

TeleopRobot::TeleopRobot(ros::NodeHandle &nh)
{
  // Load robot description from parameter server
  std::string robot_desc_string;
  nh.param("robot_description", robot_desc_string, std::string());

  // Load URDF from robot description
  if (!urdf.initString(robot_desc_string)) {
    ROS_ERROR("Failed to parse URDF from robot description");
  }

  // Parse URDF using kdl parser
  if (!kdl_parser::treeFromUrdfModel(urdf, kdl)) {
    ROS_ERROR("Failed to construct KDL from URDF");
  }
}

