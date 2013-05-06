/**
 * Creates a node that reads PCL2 data and streams it over a TCP socket
 * to any particular listeners.
 */

#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "mongo/bson/bson.h"
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/locks.hpp>
#include <boost/foreach.hpp>
#include <vector>

using boost::asio::ip::tcp; 

std::vector<boost::shared_ptr<tcp::iostream> > _streams;
boost::mutex _stream_lock;

void broadcast(mongo::BSONObj obj) 
{
    boost::lock_guard<boost::mutex> lock(_stream_lock);
    BOOST_FOREACH( boost::shared_ptr<tcp::iostream> stream, _streams ) {
      int size = obj.objsize();
      stream->write(reinterpret_cast<const char *>(&size), sizeof(int)); 
      stream->write(obj.objdata(), size);
      stream->flush();
    }
}

void pclCallback(const sensor_msgs::PointCloudConstPtr msg) 
{
  // TODO: create a BSON message and send it out
  mongo::BSONObjBuilder b;
  b.append("seq", msg->header.seq);
  b.append("frame_id", msg->header.frame_id);

  std::vector<float> points;
  BOOST_FOREACH( geometry_msgs::Point32 point, msg->points ) {
    points.push_back(point.x);
    points.push_back(point.y);
    points.push_back(point.z);
  }
  b.appendBinData("points", 
		  points.size()*sizeof(float),
		  mongo::BinDataGeneral,
		  points.data());
  
  mongo::BSONObjBuilder bson_channels;
  BOOST_FOREACH( sensor_msgs::ChannelFloat32 channel, msg->channels ) {
    bson_channels.appendBinData(channel.name, 
				channel.values.size()*sizeof(float),
				mongo::BinDataGeneral,
				channel.values.data());
  }
  b.append("channels", bson_channels.obj());

  mongo::BSONObj bson_obj = b.obj();

  // Send this data to each client
  broadcast(bson_obj);
}

void serverThreadFn(boost::asio::io_service& io_service, int port) 
{
  tcp::endpoint endpoint(tcp::v4(), port);
  tcp::acceptor acceptor(io_service, endpoint);
  acceptor.set_option(boost::asio::ip::tcp::no_delay(false));
  
  while (ros::ok()) {
    boost::shared_ptr<tcp::iostream> stream = boost::make_shared<tcp::iostream>();
    boost::system::error_code ec;    
    acceptor.accept(*stream->rdbuf(), ec);

    if (ec) {
      ROS_WARN("Connection failed: %s", ec.message().c_str());
      continue;
    } else {
      boost::lock_guard<boost::mutex> lock(_stream_lock);
      _streams.push_back(stream);
    }    
  }
}

int main(int argc, char **argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "pcl_stream");
  ros::NodeHandle nh;

  // Subscribe to a point cloud stream
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud>("points", 10, pclCallback);

  // Open a TCP socket
  int stream_port;
  nh.param<int>("port", stream_port, 6666);
  boost::asio::io_service io_service;
  boost::thread server_thread(serverThreadFn, boost::ref(io_service), stream_port);

  // Update ROS
  ROS_INFO("Starting BSON stream: %s -> [%d]", 
	   nh.resolveName("points").c_str(), stream_port);
  ros::spin();
  ROS_INFO("Stopping BSON stream: %s -> [%d]", 
	   nh.resolveName("points").c_str(), stream_port);

  // Shutdown boost.asio
  io_service.stop();
}
