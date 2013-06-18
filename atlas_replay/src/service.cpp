#include <math.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <atlas_msgs/AtlasCommand.h>
#include <sandia_hand_msgs/SimpleGrasp.h>
#include "atlas_replay/Upload.h"
#include "atlas_replay/Play.h"
#include <boost/shared_array.hpp>
#include <boost/foreach.hpp>
#include <fstream>
#include "joints.h"

ros::NodeHandle *nh_;
ros::NodeHandle *nh_private_;
ros::Publisher pub_atlas_command_;
ros::Publisher pub_left_command_;
ros::Publisher pub_right_command_;

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
  command.k_effort.resize(n);
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
    command.k_effort[i]    = 0;
    command.kp_velocity[i]  = 0; // TODO: should this be non-zero?
  }

  // Point the neck down
  command.position[3] = 0.3;
  command.k_effort[3] = 255;

  command.header.stamp = ros::Time::now();
  return command;
}

inline std::string name_trajectory(uint8_t slot) {
  return boost::str(boost::format("traj_%u.dat") % (int)slot);
}

void save_trajectory(atlas_replay::Upload::Request &trajectory) {
  size_t serial_size = ros::serialization::serializationLength(trajectory);
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);

  ros::serialization::OStream stream(buffer.get(), serial_size);
  ros::serialization::serialize<atlas_replay::Upload::Request>(stream, trajectory);

  std::ofstream traj_file(name_trajectory(trajectory.slot), std::ios::binary);
  traj_file.write((const char *)buffer.get(), serial_size);
  traj_file.close();
}

atlas_replay::Upload::Request load_trajectory(uint8_t slot) {
  atlas_replay::Upload::Request trajectory;
  trajectory.slot = slot;

  // Open trajectory file from specified slot
  std::ifstream traj_file(name_trajectory(slot), std::ios::binary);
  if (!traj_file) {
    ROS_ERROR("No trajectory in slot [%u]: %s",
              slot, name_trajectory(slot).c_str());
    return trajectory;
  }
  
  // Get size of trajectory file
  traj_file.seekg(0, traj_file.end);
  size_t serial_size = traj_file.tellg();
  traj_file.seekg(0, traj_file.beg);

  // Read file into byte buffer
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
  traj_file.read((char*)buffer.get(), serial_size);
  traj_file.close();

  // Deserialize file using ROS
  ros::serialization::IStream stream(buffer.get(), serial_size);
  ros::serialization::deserialize<atlas_replay::Upload::Request>(stream, trajectory);

  return trajectory;
}

std::vector<int> get_trajectory_joints(atlas_replay::Upload::Request &trajectory)
{
  std::vector<int> joint_indices;
  
  for (size_t limb_idx = 0; limb_idx < LIMBS.size(); ++limb_idx)
  {
    if (trajectory.flags & (1 << limb_idx))
    {
      BOOST_FOREACH(int i, LIMBS[limb_idx])
      {
        joint_indices.push_back(i);
      }
    }
  }

  return joint_indices;
}

bool play_trajectory(atlas_replay::Upload::Request &trajectory) {
  std::vector<float> joints = trajectory.commands;
  std::vector<int> joint_idx = get_trajectory_joints(trajectory);

  // Create new Atlas command with current gains
  atlas_msgs::AtlasCommand command = initialize_command();

  // Create new Sandia hand command
  sandia_hand_msgs::SimpleGrasp grasp;
  grasp.name = "cylindrical";
  
  // Execute timed trajectory
  int rate;
  nh_private_->param("rate", rate, 100);
  ros::Rate r(rate);

  // Execute timed trajectory
  ROS_INFO("Playing [%u]", trajectory.slot);
  ros::Time start = ros::Time::now();
  float end = (1.0 / trajectory.RATE) * (joints.size() / joint_idx.size() - 1);
  for (float time = 0.0; time < end;
       time = (ros::Time::now() - start).toSec())
  {
    // Get neighboring joint states for current time
    int prev_idx = (int)(time * trajectory.RATE);
    float prev_time = (1.0 / trajectory.RATE) * prev_idx;
    int next_idx = prev_idx + 1;
    float next_time = (1.0 / trajectory.RATE) * next_idx;

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
      if (joint_idx[idx] < command.position.size())
      {
        // Fill in normal joint
        command.position[joint_idx[idx]] = (nalpha * prev_state[idx] +
                                            alpha * next_state[idx]);
        command.k_effort[joint_idx[idx]] = 255;
      }
      else if (joint_idx[idx] == 28)
      {
        // Send left hand command
        grasp.closed_amount = (nalpha * prev_state[idx] +
                               alpha * next_state[idx]);
        grasp.closed_amount += 1.5708;
        grasp.closed_amount /= 3.1416;
        pub_left_command_.publish(grasp);
      }
      else if (joint_idx[idx] == 29)
      {
        // Send right hand command
        grasp.closed_amount = (nalpha * prev_state[idx] +
                               alpha * next_state[idx]);
        grasp.closed_amount += 1.5708;
        grasp.closed_amount /= 3.1416;
        pub_right_command_.publish(grasp);
      }
    }

    // Send it to Atlas!
    pub_atlas_command_.publish(command);

    // Wait until next timestep
    r.sleep();
  }
  
  ROS_INFO("Completed [%u]", trajectory.slot);
  return true;
}

bool play_service(atlas_replay::Play::Request &request,
                  atlas_replay::Play::Response &response) {
  ROS_INFO("Starting play...");
  BOOST_FOREACH(uint8_t slot, request.slots) {
    atlas_replay::Upload::Request trajectory = load_trajectory(slot);
    if (!play_trajectory(trajectory))
      return false;
  }
  return true;
  ROS_INFO("Finished play.");
}

bool upload_service(atlas_replay::Upload::Request &request,
                    atlas_replay::Upload::Request &response) {
  if (request.slot > 0) {
    ROS_INFO("Saving trajectory %u", request.slot);
    save_trajectory(request);
  }
  
  if (request.flags & (1 << request.EXECUTE)) {
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
  pub_left_command_
      = nh_->advertise<sandia_hand_msgs::SimpleGrasp>("/sandia_hands/l_hand/simple_grasp", 1);
  pub_right_command_
      = nh_->advertise<sandia_hand_msgs::SimpleGrasp>("/sandia_hands/r_hand/simple_grasp", 1);

  // Send initial command to get robot to starting pose
  for (int i = 0; i < 100; ++i) {
    atlas_msgs::AtlasCommand command = initialize_command();
    pub_atlas_command_.publish(command);
  }

  // Enter ROS main loop
  ROS_INFO("Replay service started.");
  ros::spin();
  ROS_INFO("Replay service stopped.");
}
