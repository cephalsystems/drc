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

bool is_recording_;
ros::Time start_time_;
atlas_replay::Upload::Request trajectory_;

void joint_state_callback(const sensor_msgs::JointStateConstPtr &msg)
{
  joint_states_ = *msg;
}

bool record_service(atlas_replay::Record::Request &request,
                    atlas_replay::Record::Response &response)
{
  if (request.record) {
    is_recording_ = false;
    
    trajectory_.commands.clear();
    trajectory_.times.clear();
    trajectory_.flags = 0;
    
    if (request.use_torso) 
      trajectory_.flags = trajectory_.USES_TORSO;

    if (request.use_left_leg) 
      trajectory_.flags = trajectory_.USES_LEFT_LEG;

    if (request.use_right_leg) 
      trajectory_.flags = trajectory_.USES_RIGHT_LEG;

    if (request.use_left_arm) 
      trajectory_.flags = trajectory_.USES_LEFT_ARM;

    if (request.use_right_arm) 
      trajectory_.flags = trajectory_.USES_RIGHT_ARM;

    start_time_ = ros::Time::now();
    is_recording_ = true;
  } else {
    is_recording_ = false;
  }
  return true;
}

bool send_service(atlas_replay::Play::Request &request,
                  atlas_replay::Play::Response &response)
{ 
  is_recording_ = false;
  
  if (trajectory_.flags == 0 || trajectory_.times.size() > 0 || request.slots.size() < 1) {
    return false;
  }
  
  if (request.slots.size() > 1 && request.slots[1] != 0) {
    trajectory_.flags = trajectory_.EXECUTE;
  }

  trajectory_.slot = request.slots[0];

  atlas_replay::Upload upload_call;
  upload_call.request = trajectory_;
  
  return upload_.call(upload_call);
}                    

int main(int argc, char** argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "atlas_replay_client");
  nh_ = new ros::NodeHandle();
  nh_private_ = new ros::NodeHandle("~");

  // Wait for timing information to become available
  while (ros::Time::now().toSec() == 0);

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
  int rate;
  nh_private_->param("rate", rate, 10);
  ros::Rate r(rate);
  
  // Enter ROS main loop
  while (ros::ok())
  {
    if (is_recording_)
    {
      // Add joint waypoint to list
      for (size_t limb_idx = 0; limb_idx < LIMBS.size(); ++limb_idx)
      {
        if (trajectory_.flags & limb_idx)
        {
          BOOST_FOREACH(int i, TORSO_JOINTS)
          {
            trajectory_.commands.push_back(joint_states_.position[i]);
          }
        }
      }

      // Store the fixed time offset
      ros::Duration dt = ros::Time::now() - start_time_;
      trajectory_.times.push_back(dt.toSec());
    }

    // Sleep for regular interval
    ros::spinOnce();
    r.sleep();
  }
}
