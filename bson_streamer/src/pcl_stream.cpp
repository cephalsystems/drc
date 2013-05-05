/**
 * Creates a node that reads PCL2 data and streams it over a TCP socket
 * to any particular listeners.
 */

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "mongo/bson/bson.h"
#include <boost/asio.hpp>
#include "boost/thread/thread.hpp"
#include "boost/thread/locks.hpp"
#include <vector>

using boost::asio::ip::tcp; 

std::vector<boost::shared_ptr<tcp::iostream> > _streams;
boost::mutex _stream_lock;

void pclCallback(const sensor_msgs::PointCloud2ConstPtr msg) 
{
  // TODO: create a BSON message and send it out

  {
    boost::lock_guard<boost::mutex> lock(_stream_lock);
    // TODO: send this to everyone.
  }
}

int main(int argc, char **argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "pcl_stream");
  ros::NodeHandle n;

  // Subscribe to a point cloud stream
  ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("points", 1, pclCallback);

  // Start a ROS handling thread
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Open a TCP socket
  boost::asio::io_service io_service;
  tcp::endpoint endpoint(tcp::v4(), 6666);
  tcp::acceptor acceptor(io_service, endpoint);

  while(ros::ok()) {
    boost::shared_ptr<tcp::iostream> stream = boost::make_shared<tcp::iostream>();
    boost::system::error_code ec;

    acceptor.accept(*stream->rdbuf(), ec);
    if (ec) {
      ROS_INFO("Connection failed: %s", ec.message().c_str());
      continue;
    } else {
      boost::lock_guard<boost::mutex> lock(_stream_lock);
      _streams.push_back(stream);
    }
  }
}

