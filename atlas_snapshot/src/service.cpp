#include <ros/ros.h>
#include "atlas_snapshot/Snapshot.h"
#include "atlas_joints.h"

ros::Publisher pub_snapshot;
ros::ServiceClient scan_client;

atlas_msgs::AtlasState state_;
sensor_msgs::Imu imu_;

/**
 * Calls service to update snapshot state with incoming message.
 */
void snapshot_callback(const std_msgs::Empty &msg)
{
  atlas_snapshot::Snapshot snapshot;
  
  // Get current laser scan point cloud
  laser_assember::AssembleScans::Request scan_request;
  if (scan_client.call(scan_request))
  {
    // TODO: serialize scan information somehow?
    ROS_INFO("Retrieved some laser scans");
  }

  // Load up current IMU value
  snapshot.imu = imu_.orientation;
      
  // Load up current joint values
  snapshot.joints = state_.position;

  // Send out assembled messages
  pub_snapshot.publish(snapshot);
}

void atlas_callback(const atlas_msgs::AtlasState &state)
{
  state_ = state;
}

void imu_callback(const sensor_msgs::Imu &imu)
{
  imu_ = imu;
}

/**
 * Main entry point.
 */
int main(int argc, char *argv[])
{
  // Initialize ROS node
  ros::init(argc, argv, "snapshot_service", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Connect to a bunch of topics
  pub_snapshot = nh.advertise<atlas_snapshot::Snapshot>("snapshot", 1);
  scan_client = nh.serviceClient<laser_assember::AssembleScans>("assemble_scans");
  
  ros::Subscriber sub_joint_state =
      nh.subscribe<atlas_msgs::AtlasState>("/atlas/atlas_state", 1,
                                           atlas_state_callback,
                                           ros::TransportHints().udp());
  ros::Subscriber sub_imu =
      nh.advertise<sensor_msgs::Imu>("/atlas/imu", 1,
                                     imu_callback,
                                     ros::TransportHints().udp());
  ros::Subscriber sub_request =
      nh.subscribe<sensor_msgs::>("snapshot_request", 1,
                                  snapshot_callback,
                                  ros::TransportHints().udp());

  // Spin until shutdown
  ros::spin();
}
