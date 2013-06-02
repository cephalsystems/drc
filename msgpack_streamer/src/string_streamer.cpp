#include <ros.h>
#include <std_msgs/String.h>

#include <boost/bind.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

using boost::asio::ip::tcp;
typedef boost::shared_ptr<tcp::socket> socket_ptr;
std::vector<socket_ptr> clients;
boost::mutex client_lock;

ros::Publisher<std_msgs::String> pub;
ros::Subscriber<std_msgs::String> sub;

void InputCallback(const std_msgs::StringConstPtr &msg)
{

}

void OutputCallback(const msgpack::object &obj)
{
  std_msgs::String msg;
  msg.data = "WOOPWOOP";
  pub.publish(msg);
}

void session(socket_ptr sock)
{
  boost::system::error_code error;
  msgpack::unpacker pk;
  
  // Enumerate new client
  {
    boost::lock_guard<boost::mutex> lock(client_lock);
    clients.push_back(sock);
  }

  // Start a loop of reading data and parsing it
  try {
    for (;;) {
      // Read data from socket
      pk.reserve_buffer(sock->available());
      size_t length = sock->read_some(boost::asio::buffer(pk.buffer()), error);
      if (error == boost::asio::error::eof)
        return; // Connection closed cleanly by peer.
      else if (error)
        throw boost::system::system_error(error); // Some other error.
      pk.buffer_consumed(length);
      
      // Attempt to parse any received objects
      msgpack::unpacked result;
      while(pk.next(&result)) {
        OutputCallback(result.get());
      }
    }
  }
  catch (std::exception& e)
  {
    std::cerr << "Exception in thread: " << e.what() << "\n";
  }

  // Unenumerate disconnected client
  {
    boost::lock_guard<boost::mutex> lock(client_lock);
    clients.erase(std::remove(clients.begin(), clients.end(), sock), clients.end());
  }
}

// Start a server to listen for connections
void server(boost::asio::io_service& io_service, short port)
{
  tcp::acceptor a(io_service, tcp::endpoint(tcp::v4(), port));
  for (;;)
  {
    socket_ptr sock(new tcp::socket(io_service));
    a.accept(*sock);
    boost::thread t(boost::bind(session, sock));
  }
}

int main(int argc, char *argv[])
{
  // Initialize ROS node
  ros::init(argc, argv, "streamer", ros::init_options::AnonymousName);
  ros::NodeHandle nh;

  // Load necessary parameters
  int port = nh.get_param("~port", 9999);

  // Start input and output topics
  pub = nh.advertise("output", 1);
  sub = nh.subscribe("input", 1, InputCallback);

  // Start socket server
  try {
    boost::asio::io_service io_service;
    server(io_service, port);
  } catch (std::exception &e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    exit(-1);
  }

  // Start processing ROS messages
  ros::spin();
}
