#include <ros/ros.h>
#include "atlas_joints.h"
#include "atlas_snapshot/Snapshot.h"
#include <sensor_msgs/Imu.h>
#include <atlas_msgs/AtlasState.h>
#include <std_msgs/Empty.h>
#include <laser_assembler/AssembleScans.h>
#include <boost/foreach.hpp>
#include <opencv2/opencv.hpp>

ros::Publisher pub_snapshot;
ros::ServiceClient scan_client;

atlas_msgs::AtlasState state_;
sensor_msgs::Imu imu_;

/**
 * Calls service to update snapshot state with incoming message.
 */
void snapshot_callback(const std_msgs::EmptyConstPtr &msg)
{
  ROS_INFO("Creating robot snapshot.");
  atlas_snapshot::Snapshot snapshot;
  
  // Get current laser scan point cloud for last few seconds
  laser_assembler::AssembleScans scan_params;
  scan_params.request.begin = ros::Time::now() - ros::Duration(10.0);
  scan_params.request.end = ros::Time::now();

  // Ask assembler to build this point cloud
  if (scan_client.call(scan_params))
  {
    const int width = atlas_snapshot::Snapshot::WIDTH;
    const int height = atlas_snapshot::Snapshot::HEIGHT;
    
    // Get reference to the point cloud
    sensor_msgs::PointCloud &cloud = scan_params.response.cloud;

    // Allocate buffer to hold a depth panorama
    cv::Mat image_buffer = cv::Mat(height, width, CV_8U, 255);

    // Iterate through each point, projecting into the appropriate pixel
    BOOST_FOREACH(const geometry_msgs::Point32 &point, cloud.points)
    {
      // Find the appropriate equiangular pixel mapping
      int j = (atan2(point.y, point.x)
               + M_PI) * ((float)width/(2*M_PI));
      int i = (atan2(point.z, sqrt(point.x*point.x + point.y*point.y))
               + M_PI) * ((float)height/(2*M_PI));
      float d = sqrt(point.x*point.x + point.y*point.y + point.z*point.z) * 100.0;

      // Store the closest point return
      if (d < image_buffer.at<uint8_t>(i,j))
      {
        image_buffer.at<uint8_t>(i,j) = d;
      }
    }
    
    // Encode image into jpg and store in snapshot
    // Based on: http://stackoverflow.com/a/9930442
    cv::vector<uchar> jpeg_buffer;
    cv::imencode(".jpg", image_buffer, jpeg_buffer, std::vector<int>());
    snapshot.image = jpeg_buffer;
  }

  // Load up current IMU value
  snapshot.imu = imu_.orientation;
      
  // Load up current joint values
  snapshot.joints = state_.position;

  // Send out assembled messages
  pub_snapshot.publish(snapshot);
  ROS_INFO("Completed robot snapshot.");
}

void atlas_state_callback(const atlas_msgs::AtlasStateConstPtr &state)
{
  state_ = (*state);
}

void imu_callback(const sensor_msgs::ImuConstPtr &imu)
{
  imu_ = (*imu);
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
  scan_client = nh.serviceClient<laser_assembler::AssembleScans>("assemble_scans");
  
  ros::Subscriber sub_joint_state = nh.subscribe("/atlas/atlas_state", 1,
                                                 atlas_state_callback,
                                                 ros::TransportHints().udp().tcp());
  ros::Subscriber sub_imu = nh.subscribe("/atlas/imu", 1,
                                         imu_callback,
                                         ros::TransportHints().udp().tcp());
  ros::Subscriber sub_request = nh.subscribe("snapshot_request", 1,
                                             snapshot_callback,
                                             ros::TransportHints().udp().tcp());

  // Spin until shutdown
  ros::spin();
}
