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

sensor_msgs::JointState joint_states;

bool is_recording;
ros::Time start_time;
atlas_replay::Upload::Request trajectory;

void joint_state_callback(const sensor_msgs::JointStateConstPtr &msg)
{
  joint_states = *msg;
}

bool record_service(atlas_replay::Record::Request &request,
                    atlas_replay::Record::Response &response)
{
  
}                    

bool preview_service(std_srvs::Empty::Request &request,
                     std_srvs::Empty::Response &response)
{
  
}

int main(int argc, char** argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "atlas_replay_client");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Wait for timing information to become available
  while (ros::Time::now().toSec() == 0);

  // Listen and publish joint states
  ros::Subscriber sub_joint_states =
      nh.subscribe("joint_states", 1, joint_state_callback,
                   ros::TransportHints.udp().tcp());
  ros::Publisher pub_joint_states =
      nh.advertise<sensor_msgs::JointState>("preview_states", 1);
  
  // Connect to service to record and upload trajectories
  ros::ServiceClient upload = nh.serviceClient<atlas_replay::Upload>("upload");
  ros::ServiceServer record = nh.advertiseService("record", record_service);

  // Set fixed time sampling
  int rate;
  nh_private.param("rate", rate, 10);
  ros::Rate r(rate);
  
  // Enter ROS main loop
  while (ros::ok())
  {
    if (is_recording)
    {
      // Add joint waypoint to list
      for (int limb_idx = 0; limb_idx < LIMBS.size(); ++limb_idx)
      {
        if (trajectory.flags & limb_idx) {
          BOOST_FOREACH(int i, TORSO_JOINTS)
          {
            trajectory.joints.push_back(joint_states.positions[i]);
          }
        }
      }

      // Store the fixed time offset
      ros::Duration dt = ros::Time::now() - start_time;
      trajectory.times.push_back(dt.toSec());
    }

    // Sleep for regular interval
    ros::spinOnce();
    r.sleep();
  }
}
