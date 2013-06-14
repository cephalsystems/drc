#include <math.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <atlas_msgs/AtlasCommand.h>
#include "atlas_replay/Upload.h"
#include "atlas_replay/Play.h"
#include <boost/shared_array.hpp>
#include <fstream>

ros::NodeHandle *nh_;
ros::Subscriber sub_atlas_state_;
ros::Publisher pub_atlas_command_;

// Standard table of Atlas joint names
const std::vector<std::string> ATLAS_JOINT_NAMES = {
  "back_lbz", "back_mby", "back_ubx", "neck_ay",
  "l_leg_uhz", "l_leg_mhx", "l_leg_lhy", "l_leg_kny", "l_leg_uay", "l_leg_lax",
  "r_leg_uhz", "r_leg_mhx", "r_leg_lhy", "r_leg_kny", "r_leg_uay", "r_leg_lax",
  "l_arm_usy", "l_arm_shx", "l_arm_ely", "l_arm_elx", "l_arm_uwy", "l_arm_mwx",
  "r_arm_usy", "r_arm_shx", "r_arm_ely", "r_arm_elx", "r_arm_uwy", "r_arm_mwx"
};

inline std::string name_trajectory(uint8_t slot) {
  return boost::str(boost::format("traj_%d.dat") % slot);
}

void save_trajectory(atlas_replay::Upload::Request &trajectory) {
  size_t serial_size = ros::serialization::serializationLength(trajectory);
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);

  ros::serialization::OStream stream(buffer.get(), serial_size);
  ros::serialization::serialize(stream, trajectory);

  std::ofstream traj_file(name_trajectory(trajectory.slot), std::ios::binary);
  traj_file.write((const char *)buffer.get(), serial_size);  
}

atlas_replay::Upload::Request load_trajectory(uint8_t slot) {
  atlas_replay::Upload::Request trajectory;

  size_t serial_size = ros::serialization::serializationLength(trajectory);
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);

  std::ifstream traj_file(name_trajectory(trajectory.slot), std::ios::binary);
  traj_file.read((char *)buffer.get(), serial_size);
  
  ros::serialization::IStream stream(buffer.get(), serial_size);
  ros::serialization::deserialize(stream, trajectory);

  return trajectory;
}

bool play_trajectory(atlas_replay::Upload::Request &trajectory) {
  ROS_INFO("Playing trajectory %u", trajectory.slot);

  ROS_INFO("Completed trajectory %u", trajectory.slot);
  return true;
}

bool play_service(atlas_replay::Play::Request &request,
                  atlas_replay::Play::Response &response) {
  uint8_t slot = request.slot;
  atlas_replay::Upload::Request trajectory = load_trajectory(slot);
  return play_trajectory(trajectory);
}

bool upload_service(atlas_replay::Upload::Request &request,
                    atlas_replay::Upload::Request &response) {
  if (request.slot > 0) {
    ROS_INFO("Saving trajectory %u", request.slot);
    save_trajectory(request);
  }
  
  if (request.flags & request.EXECUTE) {
    return play_trajectory(request);
  } else {
    return true;
  }
}

int main(int argc, char** argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "atlas_replay");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Wait for timing information to become available
  ros::Time last_ros_time_;
  bool wait = true;
  while (wait)
  {
    last_ros_time_ = ros::Time::now();
    if (last_ros_time_.toSec() > 0)
      wait = false;
  }
  
  // Create a service to play trajectories
  ros::ServiceServer upload = nh.advertiseService("upload", upload_service);
  ros::ServiceServer play = nh.advertiseService("play", play_service);

  // Enter ROS main loop
  ros::spin();
}
