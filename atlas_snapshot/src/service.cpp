#include <ros/ros.h>
#include "atlas_joints.h"
#include "atlas_snapshot/Snapshot.h"
#include <sensor_msgs/Imu.h>
#include <atlas_msgs/AtlasState.h>
#include <std_msgs/Empty.h>
#include <laser_assembler/AssembleScans.h>
#include <boost/foreach.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>

ros::Publisher pub_snapshot_;
ros::ServiceClient scan_client_;
tf::TransformListener *tf_listener_;

atlas_msgs::AtlasState state_;
sensor_msgs::Imu imu_;

class Camera {
 public:
  cv::Mat image;
  image_geometry::PinholeCameraModel model;
};

std::map<std::string, Camera> cameras_;
typedef std::map<std::string, Camera>::value_type camera_entry_t;

inline double remap(double input,
                    double old_min, double old_max,
                    double new_min, double new_max)
{
  double old_range = (old_max - old_min);
  double new_range = (new_max - new_min);
  return (input - old_min) / old_range * new_range + new_min;
}

/**
 * Stores last received camera info.
 */
void image_callback(const sensor_msgs::ImageConstPtr& image_msg,
                    const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  // Convert camera image
  cv_bridge::CvImagePtr input_bridge;
  try {
    input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
    cameras_[image_msg->header.frame_id].image = input_bridge->image;
  }
  catch (cv_bridge::Exception& ex){
    ROS_ERROR("[draw_frames] Failed to convert image");
    return;
  }

  // Store camera model
  cameras_[image_msg->header.frame_id].model.fromCameraInfo(info_msg);
}

/**
 * Calls service to update snapshot state with incoming message.
 */
void snapshot_callback(const std_msgs::EmptyConstPtr &msg)
{
  ROS_INFO("Creating robot snapshot.");
  atlas_snapshot::Snapshot snapshot;

  // Define fixed offset from origin for image center
  float offset[] = {0.0, 0.0, 1.0};
  
  // Get current laser scan point cloud for last few seconds
  laser_assembler::AssembleScans scan_params;
  scan_params.request.begin = ros::TIME_MIN;
  scan_params.request.end = ros::TIME_MAX;

  // Ask assembler to build this point cloud
  if (scan_client_.call(scan_params))
  {
    const int width = atlas_snapshot::Snapshot::WIDTH;
    const int height = atlas_snapshot::Snapshot::HEIGHT;
    const double fovx = atlas_snapshot::Snapshot::FOVX;
    const double fovy = atlas_snapshot::Snapshot::FOVY;
    
    // Get reference to the point cloud
    const sensor_msgs::PointCloud &cloud = scan_params.response.cloud;

    // Allocate buffer to hold image and depth panoramas
    cv::Mat image_buffer = cv::Mat(height, width, CV_8U, cv::Scalar(0));
    cv::Mat depth_buffer = cv::Mat(height, width, CV_8U, cv::Scalar(0));

    // Colorize points in cloud using all cameras
    sensor_msgs::PointCloud camera_cloud;
    std::vector<uint8_t> colors(cloud.points.size(), 0);

    BOOST_FOREACH(const camera_entry_t &entry, cameras_)
    {
      // Map point cloud to camera frame
      tf_listener_->transformPointCloud(entry.first, cloud, camera_cloud);
      
      // Iterate through each point, checking if match
      for (size_t point_idx = 0; point_idx < cloud.points.size(); ++point_idx)
      {
        // Compute color from intensity in camera image
        geometry_msgs::Point32 point = camera_cloud.points[point_idx];
        cv::Point3d point_cv(point.x, point.y, point.z);
        cv::Point2d point_uv = entry.second.model.project3dToPixel(point_cv);
        if (point_uv.x >=0 && point_uv.x < entry.second.image.cols
            && point_uv.y >= 0 && point_uv.y < entry.second.image.rows)
        {
          colors[point_idx] = entry.second.image.at<uint8_t>(point_uv.y, point_uv.x);
        }
      }
    }
    
    // Iterate through each point, projecting into the appropriate pixel
    for (size_t point_idx = 0; point_idx < cloud.points.size(); ++point_idx)
    {
      // Retrieve point location
      geometry_msgs::Point32 point = cloud.points[point_idx];
      point.x -= offset[0];
      point.y -= offset[1];
      point.z -= offset[2];
      
      // Find the appropriate equiangular pixel mapping
      double j = remap(atan2(point.y, point.x),
                       -fovx/2.0, fovx/2.0,
                       (double)width, 0);
      double i = remap(point.z / sqrt(point.x*point.x + point.y*point.y + 1e-8),
                       -tan(fovy/2.0), tan(fovy/2.0),
                       (double)height, 0);
      double d = remap(1.0 / sqrt(point.x*point.x + point.y*point.y),
                       (1.0/30.0), (1.0/0.25),
                       0, 255);

      // Store the closest point return
      if (d > depth_buffer.at<uint8_t>((int)i,(int)j))
      {
        cv::circle(depth_buffer, cv::Point(j,i), 2, (uint8_t)d, -1);
        cv::circle(image_buffer, cv::Point(j,i), 2, colors[point_idx], -1);
      }
    }
    
    // Encode image into jpg and store in snapshot
    // Based on: http://stackoverflow.com/a/9930442
    cv::vector<uchar> jpeg_image_buffer;
    cv::imencode(".jpg", image_buffer, jpeg_image_buffer, std::vector<int>());
    snapshot.image = jpeg_image_buffer;

    // Encode image into jpg and store in snapshot
    // Based on: http://stackoverflow.com/a/9930442
    cv::vector<uchar> jpeg_depth_buffer;
    cv::imencode(".jpg", depth_buffer, jpeg_depth_buffer, std::vector<int>());
    snapshot.depth = jpeg_depth_buffer;
  }

  // Load up current IMU value
  snapshot.imu = imu_.orientation;
      
  // Load up current joint values
  snapshot.joints = state_.position;

  // Send out assembled messages
  pub_snapshot_.publish(snapshot);
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
  pub_snapshot_ = nh.advertise<atlas_snapshot::Snapshot>("snapshot", 1);
  scan_client_ = nh.serviceClient<laser_assembler::AssembleScans>("assemble_scans");
  
  ros::Subscriber sub_joint_state = nh.subscribe("/atlas/atlas_state", 1,
                                                 atlas_state_callback,
                                                 ros::TransportHints().udp().tcp());
  ros::Subscriber sub_imu = nh.subscribe("/atlas/imu", 1,
                                         imu_callback,
                                         ros::TransportHints().udp().tcp());
  ros::Subscriber sub_request = nh.subscribe("snapshot_request", 1,
                                             snapshot_callback,
                                             ros::TransportHints().udp().tcp());

  tf_listener_ = new tf::TransformListener();
  image_transport::ImageTransport it(nh);
  image_transport::CameraSubscriber sub_left = it.subscribeCamera("/multisense_sl/camera/left/image_mono", 1, image_callback);
  image_transport::CameraSubscriber sub_right = it.subscribeCamera("/multisense_sl/camera/right/image_mono", 1, image_callback);
  
  // Spin until shutdown
  ROS_INFO("Started snapshot service.");
  ros::spin();
  ROS_INFO("Stopped snapshot service.");
}
