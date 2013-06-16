#include <math.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/shared_array.hpp>
#include <boost/foreach.hpp>
#include <sensor_msgs/JointState.h>
#include "atlas_replay/Upload.h"
#include "atlas_replay/Record.h"
#include "atlas_replay/Play.h"
#include <fstream>
#include "joints.h"

ros::NodeHandle *nh_;
ros::NodeHandle *nh_private_;
ros::ServiceClient upload_;

sensor_msgs::JointState joint_states_;

atlas_replay::Upload::Request trajectory_;
bool is_recording_ = false;
uint8_t record_flags_ = 0;
typedef std::map<int, float> joint_vector_t;
std::vector<joint_vector_t> recorded_states_;
size_t timestep_ = 0;

void joint_state_callback(const sensor_msgs::JointStateConstPtr &msg)
{
  joint_states_ = *msg;
}

bool record_service(atlas_replay::Record::Request &request,
                    atlas_replay::Record::Response &response)
{
  if (request.record) {
    
    // Stop previous recording session
    is_recording_ = false;
    record_flags_ = 0;
    timestep_ = 0;

    // Determine which parts are being used for this session
    if (request.torso) { 
      record_flags_ |= trajectory_.USES_TORSO;
    }

    if (request.left_leg) {
      record_flags_ |= trajectory_.USES_LEFT_LEG;
    }

    if (request.right_leg) {
      record_flags_ |= trajectory_.USES_RIGHT_LEG;
    }

    if (request.left_arm) {
      record_flags_ |= trajectory_.USES_LEFT_ARM;
    }

    if (request.right_arm) {
      record_flags_ |= trajectory_.USES_RIGHT_ARM;
    }
    
    // Combine all the parts for the trajectory
    trajectory_.flags |= record_flags_;

    // Start new recording session
    is_recording_ = true;
  } else {
    
    // Stop recording session
    is_recording_ = false;
  }
  return true;
}

void clear_trajectory(atlas_replay::Upload::Request &trajectory)
{
  recorded_states_.clear();
  trajectory_.commands.clear();
  trajectory_.flags = 0;
  timestep_ = 0;
}

bool send_service(atlas_replay::Play::Request &request,
                  atlas_replay::Play::Response &response)
{
  // Unsafe while recording
  if (is_recording_) {
    return false;
  }

  // Convert recorded states to trajectory commands
  BOOST_FOREACH(const joint_vector_t &joints, recorded_states_) {
    BOOST_FOREACH(const joint_vector_t::value_type &joint, joints) {
      trajectory_.commands.push_back(joint.second);
    }
  }
  
  // Ignore zero length or canceled trajectories (no upload AND no execution)
  if (trajectory_.flags == 0 || trajectory_.commands.size() > 0 || request.slots.size() < 1) {
    clear_trajectory(trajectory_);
    return true;
  }

  // Fill in save slot number and execute flag
  trajectory_.slot = request.slots[0];
  if (request.slots.size() > 1 && request.slots[1] != 0) {
    trajectory_.flags = trajectory_.EXECUTE;
  }

  // Upload trajectory and clear local copy
  atlas_replay::Upload upload_call;
  upload_call.request = trajectory_;
  clear_trajectory(trajectory_);

  // Return result of upload
  return upload_.call(upload_call);
}                    

int main(int argc, char** argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "atlas_replay_client");
  nh_ = new ros::NodeHandle();
  nh_private_ = new ros::NodeHandle("~");

  // Listen and publish joint states
  ros::Subscriber sub_joint_states =
      nh_->subscribe("joint_states", 1, joint_state_callback,
                     ros::TransportHints().udp().tcp());
  ros::Publisher pub_joint_states =
      nh_->advertise<sensor_msgs::JointState>("preview_states", 1);
  
  // Connect to service to record and upload trajectories
  upload_ = nh_->serviceClient<atlas_replay::Upload>("upload");
  ros::ServiceServer record = nh_->advertiseService("record", record_service);
  ros::ServiceServer send = nh_->advertiseService("send", send_service);

  // Set fixed time sampling
  ROS_INFO("Recording at %u Hz", trajectory_.RATE);
  ros::Rate r(trajectory_.RATE);
  
  // Enter ROS main loop
  while (ros::ok())
  {    
    // Sleep for regular interval
    ros::spinOnce();
    r.sleep();

    // Do nothing if we are not recording
    if (!is_recording_) {
      continue;
    }

    // Retrieve existing joints if they exist
    joint_vector_t joints;
    if (recorded_states_.size() > timestep_) {
      joints = recorded_states_[timestep_];
    }
    
    for (size_t limb_idx = 0; limb_idx < LIMBS.size(); ++limb_idx)
    {
      if (record_flags_ & limb_idx)
      {
        BOOST_FOREACH(int joint_idx, LIMBS[limb_idx])
        {
          // See if this joint is in the joint state message
          std::vector<std::string>::const_iterator pos_it
              = std::find(joint_states_.name.begin(),
                          joint_states_.name.end(),
                          ATLAS_JOINT_NAMES[joint_idx]);

          // Fill in the joint value from the message, or zero it
          joints[joint_idx] = (pos_it == joint_states_.name.end())
              ? 0.0 : joint_states_.position[joint_idx];
        }
      }
    }
    
    // Add joints to the trajectory array
    if (recorded_states_.size() > timestep_) {
      recorded_states_[timestep_] = joints;
    } else {
      recorded_states_.push_back(joints);
    }
    
    // Publish the prerecorded joint states for the existing trajectory
    sensor_msgs::JointState state;
    for (size_t joint_idx = 0; joint_idx < ATLAS_JOINT_NAMES.size(); ++joint_idx)
    {
      // Add the joint value, if it was recorded
      joint_vector_t::const_iterator joint_it = joints.find(joint_idx);
      if (joint_it != joints.end()) {
        state.name.push_back(ATLAS_JOINT_NAMES[joint_idx]);      
        state.position.push_back((*joint_it).second);
      }
    }

    // Increment to next timestep
    timestep_++;
  }
}
