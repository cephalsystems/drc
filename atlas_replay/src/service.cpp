#include <math.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <atlas_msgs/AtlasCommand.h>
#include "atlas_replay/Upload.h"
#include "atlas_replay/Play.h"
#include <boost/shared_array.hpp>
#include <boost/foreach.hpp>
#include <fstream>
#include "joints.h"

ros::NodeHandle *nh_;
ros::NodeHandle *nh_private_;
ros::Publisher pub_atlas_command_;

// Initialze Atlas command with current gains and correct size
atlas_msgs::AtlasCommand initialize_command() {
  atlas_msgs::AtlasCommand command;

  unsigned int n = ATLAS_JOINT_NAMES.size();
  command.position.resize(n);
  command.velocity.resize(n);
  command.effort.resize(n);
  command.kp_position.resize(n);
  command.ki_position.resize(n);
  command.kd_position.resize(n);
  command.kp_velocity.resize(n);
  command.i_effort_min.resize(n);
  command.i_effort_max.resize(n);

  // Load current gains from ROS master
  for (unsigned int i = 0; i < n; i++) {
    double kp;
    nh_->getParam("atlas_controller/gains/" + ATLAS_JOINT_NAMES[i] + "/p", kp);
    command.kp_position[i] = (float)kp;

    double ki;
    nh_->getParam("atlas_controller/gains/" + ATLAS_JOINT_NAMES[i] + "/i", ki);
    command.ki_position[i] = (float)ki;

    double kd;
    nh_->getParam("atlas_controller/gains/" + ATLAS_JOINT_NAMES[i] + "/d", kd);
    command.kd_position[i] = (float)kd;

    double i_clamp;
    nh_->getParam("atlas_controller/gains/" + ATLAS_JOINT_NAMES[i] + "/i_clamp", i_clamp);
    command.i_effort_min[i] = -i_clamp;
    command.i_effort_max[i] = i_clamp;

    command.effort[i]       = 0;
    command.kp_velocity[i]  = 0; // TODO: should this be non-zero?
  }

  return command;
}

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

std::vector<int> get_trajectory_joints(atlas_replay::Upload::Request &trajectory)
{
  std::vector<int> joint_indices;
  
  for (size_t limb_idx = 0; limb_idx < LIMBS.size(); ++limb_idx)
  {
    if (trajectory.flags & limb_idx)
    {
      BOOST_FOREACH(int i, TORSO_JOINTS)
      {
        joint_indices.push_back(i);
      }
    }
  }

  return joint_indices;
}

bool play_trajectory(atlas_replay::Upload::Request &trajectory) {
  std::vector<float> times = trajectory.times;
  std::vector<float> joints = trajectory.commands;
  std::vector<int> joint_idx = get_trajectory_joints(trajectory);

  // Create new Atlas command with current gains
  atlas_msgs::AtlasCommand command = initialize_command();
  
  // Execute timed trajectory
  int rate;
  nh_private_->param("rate", rate, 100);
  ros::Rate r(rate);

  // Execute timed trajectory
  ROS_INFO("Playing trajectory %u", trajectory.slot);
  ros::Time start = ros::Time::now();
  for (float time = 0.0; time < times.back();
       time = (ros::Time::now() - start).toSec())
  {
    // Get neighboring joint states for current time
    std::vector<float>::iterator prev_it, next_it;
    prev_it = std::lower_bound(times.begin(), times.end(), time);
    float prev_time = *prev_it;
    int prev_idx = prev_it - times.begin();
    
    next_it = std::upper_bound(times.begin(), times.end(), time);
    float next_time = *next_it;
    int next_idx = next_it - times.begin();

    // Load previous and next joint states
    std::vector<float> prev_state(joints.begin() + prev_idx * joint_idx.size(),
                                  joints.begin() + (prev_idx + 1) * joint_idx.size());
    std::vector<float> next_state(joints.begin() + next_idx * joint_idx.size(),
                                  joints.begin() + (next_idx + 1) * joint_idx.size());
    
    // Interpolate to get intermediate state
    float alpha = (time - prev_time) / (next_time - prev_time);
    float nalpha = 1 - alpha;

    // Create interpolated command
    for (size_t idx = 0; idx < joint_idx.size(); ++idx) {
      command.position[joint_idx[idx]] = (nalpha * prev_state[idx] +
                                          alpha * next_state[idx]);
      command.effort[joint_idx[idx]] = 255;
    }

    // Send it to Atlas!
    pub_atlas_command_.publish(command);

    // Wait until next timestep
    r.sleep();
  }
  
  ROS_INFO("Completed trajectory %u", trajectory.slot);
  return true;
}

bool play_service(atlas_replay::Play::Request &request,
                  atlas_replay::Play::Response &response) {
  BOOST_FOREACH(uint8_t slot, request.slots) {
    atlas_replay::Upload::Request trajectory = load_trajectory(slot);
    if (!play_trajectory(trajectory))
      return false;
  }
  return true;
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
  ros::init(argc, argv, "atlas_replay_service");
  nh_ = new ros::NodeHandle();
  nh_private_ = new ros::NodeHandle("~");

  // Wait for timing information to become available
  while (ros::Time::now().toSec() == 0);
  
  // Create a service to play trajectories
  ros::ServiceServer upload = nh_->advertiseService("upload", upload_service);
  ros::ServiceServer play = nh_->advertiseService("play", play_service);

  // Create a publisher to send commands to robot
  pub_atlas_command_
      = nh_->advertise<atlas_msgs::AtlasCommand>("/atlas/atlas_command", 1);

  // Enter ROS main loop
  ros::spin();
}
