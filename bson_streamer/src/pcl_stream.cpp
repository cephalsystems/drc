/**
 * Creates a node that reads PCL2 data and streams it over a TCP socket
 * to any particular listeners.
 */

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "mongo/bson/bson.h"
#include "boost/thread/thread.hpp"
#include "boost/thread/locks.hpp"
#include <vector>

using boost::asio::ip::tcp; 

std::vector<tcp::iostream> _streams;
boost::mutex _stream_lock;

void pclCallback(const sensor_msgs::PointCloud2& msg) 
{
  // TODO: create a BSON message and send it out

  {
    boost::lock_guard<boost::mutex>(_stream_lock);
    // TODO: send this to everyone.
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcl_stream");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("points", pclCallback);
  // TODO: run an async spinner here

  // Open a TCP socket
  boost::asio::io_service io_service;
  tcp::endpoint endpoint(tcp::v4(), 6666);
  tcp::acceptor acceptor(io_service, endpoint);

  while(ros::ok()) {
    tcp::iostream stream;
    boost::system::error_code ec;

    acceptor.accept(*stream.rdbuf(), ec);
    if (ec) {
      ROS_INFO("Connection failed: %d", ec);
      continue;
    } else {
      boost::lock_guard<boost::mutex> lock(_stream_lock);
      _streams.push_back(stream);
    }
  }
}

