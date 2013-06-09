/**
 * Creates a node that reads PCL2 data and streams it over a TCP socket
 * to any particular listeners.
 */

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <msgpack.hpp>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/locks.hpp>
#include <boost/foreach.hpp>
#include <vector>

using boost::asio::ip::tcp; 

std::vector<boost::shared_ptr<tcp::iostream> > _streams;
boost::mutex _stream_lock;

void broadcast(msgpack::sbuffer &buffer) 
{
    boost::lock_guard<boost::mutex> lock(_stream_lock);
    BOOST_FOREACH( boost::shared_ptr<tcp::iostream> stream, _streams ) {
      stream->write(buffer.data(), buffer.size());
      stream->flush();
    }
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
  ros::init(argc, argv, "msgpack_tf_stream");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  // Get a world frame
  std::string fixed_frame = "/world";
  nhp.getParam("fixed_frame", fixed_frame);

  // Get a list of all frames of interest
  XmlRpc::XmlRpcValue frame_list;
  nhp.getParam("frames", frame_list);
  ROS_ASSERT(frame_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  std::vector<std::string> frames;
  for (int32_t i = 0; i < frame_list.size(); ++i) {
    ROS_ASSERT(frame_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    frames.push_back(static_cast<std::string>(frame_list[i]));
  }

  // Subscribe to the transform tree
  tf::TransformListener listener;

  // Open a TCP socket
  int stream_port;
  nhp.param<int>("port", stream_port, 6666);
  boost::asio::io_service io_service;
  boost::thread server_thread(serverThreadFn, boost::ref(io_service), stream_port);

  // Update ROS and send out TFs at fixed-rate
  ROS_INFO("Starting MSGPACK TF stream.");
  ros::Rate rate(10.0);
  while (nh.ok()) {

    // Convert each frame to vector
    std::map<std::string, std::vector<float> > frame_entries;
    BOOST_FOREACH( std::string frame, frames ) {
      try{
	tf::StampedTransform transform;
	listener.lookupTransform(frame, fixed_frame, ros::Time(0), transform);

	std::vector<float> frame_entry;
	frame_entry.push_back(transform.getOrigin().x());
	frame_entry.push_back(transform.getOrigin().y());
	frame_entry.push_back(transform.getOrigin().z());
	frame_entry.push_back(transform.getRotation().x());
	frame_entry.push_back(transform.getRotation().y());
	frame_entry.push_back(transform.getRotation().z());
	frame_entry.push_back(transform.getRotation().w());

	frame_entries[frame] = frame_entry;
      }
      catch (tf::TransformException ex){
	ROS_ERROR("%s",ex.what());
      }
    }

    // Serialize and send out new frames
    msgpack::sbuffer buffer;
    msgpack::packer<msgpack::sbuffer> pk(&buffer);
    pk.pack(frame_entries);
    broadcast(buffer);

    // Sleep until next cycle
    ros::spinOnce();  
    rate.sleep();
  }
  ROS_INFO("Stopping MSGPACK TF stream.");

  // Shutdown boost.asio
  io_service.stop();
}
