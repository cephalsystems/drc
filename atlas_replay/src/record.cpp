#include <math.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <atlas_msgs/AtlasState.h>

ros::NodeHandle *nh_;
ros::Subscriber<atlas_msgs::AtlasState> sub_atlas_state_;

volatile bool is_recording_ = false;
ros::Time recording_start_time_;
Execute::Request trajectory_;

// Standard table of Atlas joint names
const std::vector<std::string> ATLAS_JOINT_NAMES = {
  "back_lbz", "back_mby", "back_ubx", "neck_ay",
  "l_leg_uhz", "l_leg_mhx", "l_leg_lhy", "l_leg_kny", "l_leg_uay", "l_leg_lax",
  "r_leg_uhz", "r_leg_mhx", "r_leg_lhy", "r_leg_kny", "r_leg_uay", "r_leg_lax",
  "l_arm_usy", "l_arm_shx", "l_arm_ely", "l_arm_elx", "l_arm_uwy", "l_arm_mwx",
  "r_arm_usy", "r_arm_shx", "r_arm_ely", "r_arm_elx", "r_arm_uwy", "r_arm_mwx"
};

void save_trajectory(Execute::Request &trajectory) {
  // TODO: fill this in
}

void Execute::Request load_trajectory(std::string filename) {
  // TODO: fill this in
}

void atlas_state_callback(const atlas_msgs::AtlasState::ConstPtr &state)
{
  if (is_recording_) {
    trajectory_.states.push_back(state);
    trajectory_.times.push_back(state.header.time - recording_start_time_);
  }
}

bool record_callback(Record::Request &req, Record::Response &resp)
{
  if (is_recording_ && !req.is_recording) {
    
    // We are done recording
    is_recording_ = false;
    
    // If filename is provided, save to file
    if (!req.name.is_empty()) {
      save_trajectory(trajectory_);
    }
    
  } else if (!is_recording && req.is_recording) {
    
    // Clear previous recording
    trajectory_.states.clear();
    trajectory_.times.clear();
    
    // Get current time and start recording
    recording_start_time_ = ros::Time::now();
    is_recording_ = true;
  }

  resp.status = "OK";
  return true;
}

bool play_callback(Play::Request& req, Play::Response& resp);
{
  atlas_replay::Execute execute;
  
  // Possibly load trajectory from file
  if (req.name.is_empty()) {
    execute.request = trajectory;
  } else {
    execute.request = load_trajectory(req.name);
  }

  // Connect to playback service and send trajectory
  if (ros::service::call("execute", execute))
  {
    response.status(execute.response.status);
    // TODO: send some sort of joint state update?
    return true;
  } else {
    response.status("Failed to execute.");
    return false;
  }
}

int main(int argc, char** argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "atlas_record");
  nh_ = new ros::NodeHandle();

  // Wait for timing information to become available
  ros::Time last_ros_time_;
  bool wait = true;
  while (wait)
  {
    last_ros_time_ = ros::Time::now();
    if (last_ros_time_.toSec() > 0)
      wait = false;
  }
  
  // Subscribe to AtlasState
  ros::SubscribeOptions atlas_state_options =
      ros::SubscribeOptions::create<atlas_msgs::AtlasState>(
          "/atlas/atlas_state", 1, atlas_state_callback,
          ros::VoidPtr(), nh_->getCallbackQueue());
  atlas_state_options.transport_hints = ros::TransportHints().unreliable();
  sub_atlas_state_ = nh_->subscribe(atlas_state_options);
  
  // Create a service to play trajectories
  ros::ServiceServer record_service = nh.advertiseService("record", record_callback);
  ros::ServiceServer record_service = nh.advertiseService("play", play_callback);

  // Enter ROS main loop at fixed frequency
  ros::Rate rate(10);
  while(nh_->isOk()) {
    ros::spinOnces;
    rate.sleep();
  }
  return 0;
}
