#include <ros/ros.h>
#include "atlas_snapshot/Snapshot.h"
#include "atlas_joints.h"
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_broadcaster.h>
#include <boost/foreach.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>

/**
 * Current robot state.
 */
sensor_msgs::Image image_;
sensor_msgs::JointState joints_;
tf::Transform imu_ = tf::Transform::getIdentity();

/**
 * Global node connections
 */
image_transport::Publisher pub_image_;

/**
 * Calls service to update snapshot state with incoming message.
 */
void snapshot_callback(const atlas_snapshot::Snapshot &msg)
{
  // Decode quaternion as new imu transformation
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg.imu, q);
  imu_.setRotation(q);

  // Decode joints in order
  joints_.name = ATLAS_JOINT_NAMES;
  joints_.position.clear();
  BOOST_FOREACH(float joint, msg.joints)
  {
    joints_.position.push_back(joint);
  }

  // Decode panorama image into regular image
  try
  {
    // Convert from intensity JPEG back to OpenCV
    const cv::Mat image_buffer = cv::Mat(msg.image);
    cv::Mat image_matrix = cv::imdecode(image_buffer, -1);

    // Convert from depth JPEG back to OpenCV
    const cv::Mat depth_buffer = cv::Mat(msg.depth);
    cv::Mat depth_matrix = cv::imdecode(depth_buffer, -1);
 
    // Create an output image that is three channels
    const std::vector<cv::Mat> in_matrices
        = {depth_matrix, image_matrix, image_matrix};
    cv::Mat rgb_matrix(image_matrix.rows, image_matrix.cols, CV_8UC3);
    cv::merge(in_matrices, rgb_matrix);
    
    // Create a simple timestamp
    std_msgs::Header header;
    header.stamp = ros::Time::now();

    // Convert from OpenCV to image message
    cv_bridge::CvImage image = cv_bridge::CvImage(header, "rgb8", rgb_matrix);
    image_ = *(image.toImageMsg());
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Send out the latest image
  if (image_.data.size() > 0) {
    image_.header.stamp = ros::Time::now();
    pub_image_.publish(image_);
  }
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
      nh.advertise<sensor_msgs::JointState>("joint_states", 1, true);
  ros::Subscriber sub_snapshot = nh.subscribe("snapshot", 1, snapshot_callback);

  image_transport::ImageTransport it(nh);
  pub_image_ = it.advertise("image", 1);
  
  // Set up fixed publish rate
  int rate;
  nh_private.param("rate", rate, 100);
  ros::Rate r(rate);
  
  // Republish the latest snapshot on regular intervals
  while(ros::ok()) {

    // Send out the latest joint state
    if (joints_.name.size() > 0) {
      joints_.header.stamp = ros::Time::now();
      pub_joint_state.publish(joints_);
    }

    // Send out the latest IMU transformation for the head
    br.sendTransform(tf::StampedTransform(imu_, ros::Time::now(),
                                          "world", "imu_link"));

    // Wait for next cycle
    ros::spinOnce();
    r.sleep();
  }
}
