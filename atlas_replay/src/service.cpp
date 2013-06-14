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
#include "joints.h"

ros::NodeHandle *nh_;
ros::Subscriber sub_atlas_state_;
ros::Publisher pub_atlas_command_;

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
  while (ros::Time::now().toSec() == 0);
  
  // Create a service to play trajectories
  ros::ServiceServer upload = nh.advertiseService("upload", upload_service);
  ros::ServiceServer play = nh.advertiseService("play", play_service);

  // Enter ROS main loop
  ros::spin();
}
