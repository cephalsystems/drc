/**
 * Base implementation of object streaming.
 * Overridden to provide varying implementations.
 */
#ifndef STREAMER_H_
#define STREAMER_H_

#include <msgpack.hpp>
#include <string>

template<typename In, typename Out>
class Streamer
{
private:
  ros::Publisher pub;
  ros::Subscriber sub;
  
public:
  Streamer(std::string inTopic, std::string outTopic)
  {
    inPub = n.advertise<Out>(outTopic, 1);
    inSub = n.subscribe<In>(inTopic, 1, inputCallback);
  };
  
  virtual ~Streamer() {};

  void inputCallback(const In* const msg)
  {
    msgpack::sbuffer buffer;
    msgpack::packer<msgpack::sbuffer> pk(&buffer);
    pack(pk);
    // Send buffer out over TCP socket
  }

  void outputThread(void *arg)
  {
    msgpack::unpacker pk;
    
    while(ros::ok()) {
      
      // Read incoming data on socket
      // pk.reserve_buffer(socket.available());
      // pk.buffer_consumed(socket.read(pk.buffer(), socket.available()))
      // TODO: do this

      // Unpack resulting message
      msg::unpacked result;
      while(pk.next(&result)) {
        msgpack::object obj = result.get();
        In input = unpack(obj);
        pub.publish(input);
      }
    }
  }
  
  virtual void pack(msgpack::packer<msgpack::sbuffer> pk, In &input) {};
  virtual In unpack(msgpack::object obj) {};
};

#endif /* STREAMER_H_ */
