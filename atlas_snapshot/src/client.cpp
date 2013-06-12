#include <ros/ros.h>
#include "atlas_snapshot/Snapshot.h"
#include "atlas_joints.h"

/**
 * Current robot state.
 */
sensor_msgs::PointCloud cloud_;
sensor_msgs::JointState joints_;
tf::Transform imu_;

/**
 * Calls service to update snapshot state with incoming message.
 */
void snapshot_callback(const atlas_snapshot::Snapshot &msg)
{
  // Decode quaternion as new imu transformation
  imu_.setQuaternion(msg.imu_);

  // Decode joints in order
  joints_.name = ATLAS_JOINT_NAMES;
  joints_.position.clear();
  BOOST_FOREACH(float joint, msg.joints)
  {
    joints_.position.push_back(joint);
  }

  // Decode point cloud... somehow?
}

/**
 * Main entry point.
 */
int main(int argc, char *argv[])
{
  // Initialize ROS node
  ros::init(argc, argv, "snapshot_client", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Connect to a bunch of topics
  tf::TransformBroadcaster br;
  ros::Publisher pub_joint_state =
      nh.advertise<sensor_msgs::JointState>("joint_states", 2);
  ros::Publisher pub_points =
      nh.advertise<sensor_msgs::JointState>("points", 2);
  ros::Subscriber sub_snapshot = nh.subscribe("snapshot", 1, snapshot_callback);

  // Set up fixed publish rate
  int rate;
  nh_private.param("rate", rate, 100);
  ros::Rate r(rate);
  
  // Republish the latest snapshot on regular intervals
  while(ros::ok()) {
    
    // Send out the latest joint state
    pub_joint_state.publish(joints_);

    // Send out the latest point cloud
    pub_points.publish(points_);

    // Send out the latest IMU transformation for the head
    br.sendTransform(tf::StampedTransform(imu_, ros::Time::now(),
                                          "world", "imu_link"));

    // Wait for next cycle
    ros::spinOnce();
    r.sleep();
  }
}
