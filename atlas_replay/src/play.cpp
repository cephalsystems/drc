#include <math.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <sensor_msgs/JointState.h>
#include <atlas_msgs/JointCommand.h>

ros::NodeHandle *nh_;
ros::Subscriber<atlas_msgs::AtlasState> sub_atlas_state_;
ros::Publisher<atlas_msgs::AtlasCommand> pub_atlas_command_;
atlas_msgs::AtlasState current_joint_state_;

// Standard table of Atlas joint names
const std::vector<std::string> ATLAS_JOINT_NAMES = {
  "back_lbz", "back_mby", "back_ubx", "neck_ay",
  "l_leg_uhz", "l_leg_mhx", "l_leg_lhy", "l_leg_kny", "l_leg_uay", "l_leg_lax",
  "r_leg_uhz", "r_leg_mhx", "r_leg_lhy", "r_leg_kny", "r_leg_uay", "r_leg_lax",
  "l_arm_usy", "l_arm_shx", "l_arm_ely", "l_arm_elx", "l_arm_uwy", "l_arm_mwx",
  "r_arm_usy", "r_arm_shx", "r_arm_ely", "r_arm_elx", "r_arm_uwy", "r_arm_mwx"
};

// Initialze Atlas command with current gains and correct size
atlas_msgs::AtlasCommand initialize_command() {
  AtlasCommand command;
  
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
    nh_->getParam("atlas_controller/gains/" + ATLAS_JOINT_NAMES[i] + "/p",
                      command.kp_position[i]);
    nh_->getParam("atlas_controller/gains/" + ATLAS_JOINT_NAMES[i] + "/i",
                      command.ki_position[i]);
    nh_->getParam("atlas_controller/gains/" + ATLAS_JOINT_NAMES[i] + "/d",
                      command.kd_position[i]);
    nh_->getParam("atlas_controller/gains/" + ATLAS_JOINT_NAMES[i] + "/i_clamp",
                      command.i_effort_min[i]);
    command.i_effort_min[i] = -command.i_effort_min[i];
    nh_->getParam("atlas_controller/gains/" + ATLAS_JOINT_NAMES[i] + "/i_clamp",
                      command.i_effort_max[i]);
    
    command.effort[i]       = 0;
    command.kp_velocity[i]  = 0; // TODO: should this be non-zero?
  }

  return command;
}

void joint_states_callback(const sensor_msgs::JointState::ConstPtr &_js)
{
  current_joint_state_ = *_js;
}

bool play_callback(Play::Request& req, Play::Response& resp);
{
  // Create new Atlas command with current gains
  atlas_msgs::AtlasCommand command = initialize_command();
  
  // Compute timing information for this trajectory
  ros::Time start_time = ros::Time::now();
  float current_time = start_time;
  ros::Time end_time = start_time + request.times.last();

  // Execute timed trajectory
  ros::Rate rate(100);
  while((current_time = ros::Time::now()) < end_time) {
    
    // Get neighboring joint states for current time
    std::vector<atlas_msgs::AtlasState>::iterator prev_it, next_it;
    prev_it = std::lower_bound(req.times.begin(), req.times.end(), current_time);
    next_it = std::upper_bound(req.times.begin(), req.times.end(), current_time);
    float prev_time = *prev_it;
    float next_time = *next_it;
    atlas_msgs::AtlasState prev_state = req.states[prev_time - req.times.begin()];
    atlas_msgs::AtlasState next_state = req.states[next_time - req.times.begin()];

    // Interpolate to get intermediate state
    float alpha = (current_time - prev_time) / (next_time - prev_time);
    float nalpha = 1 - alpha;

    // Create interpolated command
    for (int idx = 0; idx < prev_state.position.size(); ++i) {
      resp.position[idx](nalpha * prev_state.position +
                         alpha * next_state.position);
      resp.velocity[idx](nalpha * prev_state.velocity +
                         alpha * next_state.velocity);
    }

    // Send it to Atlas!
    pub_atlas_command_.publish(command);
        
    // Wait until next timestep
    rate.sleep();
  }

  // Return status indicating current state of robot
  response.state = current_joint_state_;
  response.status = "OK";
  return true;
}

int main(int argc, char** argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "atlas_play");
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
  
  // Publish to AtlasCommand
  pub_atlas_command_ = nh_->advertise("/atlas/atlas_command", 1, true);

  // Create a service to play trajectories
  ros::ServiceServer play_service = nh.advertiseService("play", play_callback);

  // Enter ROS main loop
  ros::AsyncSpinner spinner(3);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
