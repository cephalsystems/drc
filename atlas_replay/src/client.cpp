#include <math.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/shared_array.hpp>
#include <boost/foreach.hpp>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include "atlas_replay/Upload.h"
#include "atlas_replay/Record.h"
#include "atlas_replay/Play.h"
#include <fstream>
#include "joints.h"
#include <urdf/model.h>
#include <iostream>

typedef std::map<int, float> joint_vector_t;
typedef std::map<std::string, boost::shared_ptr<urdf::Joint> > joint_map_t;

ros::NodeHandle *nh_;
ros::NodeHandle *nh_private_;
ros::ServiceClient upload_;

sensor_msgs::JointState joint_states_;

atlas_replay::Upload::Request trajectory_;
bool is_recording_ = false;
uint8_t record_flags_ = 0;
std::vector<joint_vector_t> recorded_states_;
size_t timestep_ = 0;

void joint_state_callback(const sensor_msgs::JointStateConstPtr &msg)
{
  joint_states_ = *msg;
}

bool record_service(atlas_replay::Record::Request &request,
                    atlas_replay::Record::Response &response)
{
  // If we are recording, fill out the rest of this track
  // with the current joint state
  if (is_recording_  && record_flags_ && timestep_ > 0) {

    // Get the last recorded state
    joint_vector_t last_joints = recorded_states_[timestep_ - 1];

    // Apply this recorded state to all future states
    for (; timestep_ < recorded_states_.size(); ++timestep_)
    {
      for (size_t limb_idx = 0; limb_idx < LIMBS.size(); ++limb_idx)
      {
        if (record_flags_ & (1 << limb_idx))
        {
          BOOST_FOREACH(int joint_idx, LIMBS[limb_idx])
          {
            (recorded_states_[timestep_])[joint_idx] = last_joints[joint_idx];
          }
        }
      }
    }
  }

  // Stop recording at this point
  is_recording_ = false;
  timestep_ = 0;
  record_flags_ = 0;

  // Determine which parts are being used for this session
  if (request.torso) { 
    record_flags_ |= 1 << trajectory_.USES_TORSO;
  }
  
  if (request.left_leg) {
    record_flags_ |= 1 << trajectory_.USES_LEFT_LEG;
  }
  
  if (request.right_leg) {
    record_flags_ |= 1 << trajectory_.USES_RIGHT_LEG;
  }
  
  if (request.left_arm) {
    record_flags_ |= 1 << trajectory_.USES_LEFT_ARM;
  }
  
  if (request.right_arm) {
    record_flags_ |= 1 << trajectory_.USES_RIGHT_ARM;
  }
  
  // If we are starting recording again, set stuff up
  if (request.record) {
    // Combine all the parts for the trajectory
    trajectory_.flags |= record_flags_;
    
    // Start new recording session
    is_recording_ = true;
  }
  
  return true;
}

void clear_trajectory(atlas_replay::Upload::Request &trajectory)
{
  recorded_states_.clear();
  trajectory_.flags = 0;
  timestep_ = 0;
}

bool send_service(atlas_replay::Play::Request &request,
                  atlas_replay::Play::Response &response)
{
  // Unsafe while recording
  if (is_recording_) {
    ROS_INFO("Cannot send while recording.");
    return false;
  }

  // Convert recorded states to trajectory commands
  trajectory_.commands.clear();
  BOOST_FOREACH(const joint_vector_t &joints, recorded_states_) {
    BOOST_FOREACH(const joint_vector_t::value_type &joint, joints) {
      trajectory_.commands.push_back(joint.second);
    }
  }
  
  // Ignore zero length or canceled trajectories (no upload AND no execution)
  if (trajectory_.flags == 0 || trajectory_.commands.size() == 0 || request.slots.size() == 0) {
    ROS_INFO("Ignoring invalid trajectory: %u %lu %lu",
             trajectory_.flags, trajectory_.commands.size(), request.slots.size());
    clear_trajectory(trajectory_);
    return true;
  }

  // Fill in save slot number and execute flag
  trajectory_.slot = request.slots[0];
  if (request.slots.size() > 1 && request.slots[1] != 0) {
    trajectory_.flags |= 1 << trajectory_.EXECUTE;
  }

  // Upload trajectory and clear local copy
  atlas_replay::Upload upload_call;
  upload_call.request = trajectory_;

  // Return result of upload
  ROS_INFO("Uploading trajectory: %u", trajectory_.slot);
  bool success = upload_.call(upload_call);
  ROS_INFO("Upload complete.");
  return success;
}                    

int main(int argc, char** argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "atlas_replay_client");
  nh_ = new ros::NodeHandle();
  nh_private_ = new ros::NodeHandle("~");

  // Retrieve current robot model
  std::string robot_description;
  if (!nh_->getParam("robot_description", robot_description))
  {
    ROS_ERROR("No robot description parameter found!");
    return -1;
  }

  // Load robot model from parameter string
  urdf::Model urdf;
  if (!urdf.initString(robot_description))
  {
    ROS_ERROR("Failed to parse the URDF");
    return -1;
  }
  
  // Listen and publish joint states
  ros::Subscriber sub_joint_states =
      nh_->subscribe("joint_states", 1, joint_state_callback,
                     ros::TransportHints().udp().tcp());
  ros::Publisher pub_joint_states =
      nh_->advertise<sensor_msgs::JointState>("preview_states", 1);
  ros::Publisher pub_joint_usage =
      nh_->advertise<std_msgs::String>("commands", 1);
  
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

    // Send out command to highlight only used joints
    std::stringstream ss;
    ss << "robot:clearHighlights()" << std::endl;
    for (size_t limb_idx = 0; limb_idx < LIMBS.size(); ++limb_idx)
    {
      if (record_flags_ & (1 << limb_idx))
      {
        BOOST_FOREACH(int joint_idx, LIMBS[limb_idx])
        {
          boost::shared_ptr<const urdf::Joint> joint
              = urdf.getJoint(AUGMENTED_ATLAS_JOINT_NAMES[joint_idx]);
          if (joint)
          {
            ss << boost::format("robot:setHighlight('/teleop/%s', true)")
                % joint->child_link_name.c_str() << std::endl;
          }
        }
      }
    
    }

    std_msgs::String highlight_command;
    highlight_command.data = ss.str();
    pub_joint_usage.publish(highlight_command);
    
    // Just forward joint states if we are not recording
    if (!is_recording_) {
      pub_joint_states.publish(joint_states_);
      continue;
    }

    // Retrieve existing joints if they exist
    joint_vector_t joints;
    if (recorded_states_.size() > timestep_) {
      joints = recorded_states_[timestep_];
    } else if (recorded_states_.size() > 0) {
      joints.insert(recorded_states_.back().begin(), recorded_states_.back().end());
    }
    
    for (size_t limb_idx = 0; limb_idx < LIMBS.size(); ++limb_idx)
    {
      if (record_flags_ & (1 << limb_idx))
      {
        BOOST_FOREACH(int joint_idx, LIMBS[limb_idx])
        {
          // See if this joint is in the joint state message
          std::vector<std::string>::const_iterator pos_it
              = std::find(joint_states_.name.begin(),
                          joint_states_.name.end(),
                          AUGMENTED_ATLAS_JOINT_NAMES[joint_idx]);

          // Fill in the joint value from the message, or zero it
          joints[joint_idx] = (pos_it == joint_states_.name.end())
              ? 0.0 : joint_states_.position[pos_it - joint_states_.name.begin()];
        }
      }
    }
    
    // Add joints to the trajectory array
    if (record_flags_) {
      if (recorded_states_.size() > timestep_) {
        recorded_states_[timestep_] = joints;
      } else {
        recorded_states_.push_back(joints);
      }
    }
    
    // Publish the prerecorded joint states for the existing trajectory
    sensor_msgs::JointState state;
    BOOST_FOREACH(const joint_map_t::value_type &joint, urdf.joints_)
    {
      // Add the name to the joint state list and assume zero value
      state.name.push_back(joint.first);
      state.position.push_back(0.0);
      
      // Search for this joint in the ATLAS JOINT VECTOR
      std::vector<std::string>::const_iterator joint_name_it
          = std::find(AUGMENTED_ATLAS_JOINT_NAMES.begin(),
                      AUGMENTED_ATLAS_JOINT_NAMES.end(), joint.first);

      // If this is an atlas joint, try to get its value
      if (joint_name_it != AUGMENTED_ATLAS_JOINT_NAMES.end())
      {
        size_t joint_idx = joint_name_it - AUGMENTED_ATLAS_JOINT_NAMES.begin();

        // Add the joint value, if it was recorded
        joint_vector_t::const_iterator joint_it = joints.find(joint_idx);
        if (joint_it != joints.end())
        {
          state.position.back() = (*joint_it).second;
        }
      }
    }

    state.header.stamp = ros::Time::now();
    pub_joint_states.publish(state);

    // Increment to next timestep
    timestep_++;
  }
}
