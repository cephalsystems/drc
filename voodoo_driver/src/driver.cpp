
#include "simpleserial.h"
#include <string>
#include <boost/foreach.hpp>

class SerialMapping
{
 public:
  std::string port_name;
  std::map<int, std::string> joint_names;
};

typedef XmlRpc::XmlRpcValue::ValueStruct::value_type struct_member;

SerialMapping LoadMapping(const std::string &port_name,
                          XmlRpc::XmlRpcValue &mapping_param)
{
  SerialMapping mapping;
  mapping.port_name = port_name;

  ROS_ASSERT(mapping_param.getType() == XmlRpc::XmlRpcValue::TypeStruct);

  XmlRpc::XmlRpcValue::iterator it;
  for (it = mapping_param.begin(); it != mapping_param.end(); ++it) {
    ROS_ASSERT((*it).second.getType() == XmlRpc::XmlRpcValue::TypeString);
    mapping.joint_names[atoi((*it).first.c_str())] = (std::string)(*it).second;
  }

  return mapping;
}

std::vector<SerialMapping> LoadMappings(const ros::NodeHandle &nh,
                                        const std::string &param_name)
{
  std::vector<SerialMapping> mappings;
  
  XmlRpc::XmlRpcValue mappings_param;
  nh.getParam(param_name, mappings_param);

  ROS_ERROR("%d", mappings_param.getType());
  ROS_ASSERT(mappings_param.getType() == XmlRpc::XmlRpcValue::TypeStruct);

  XmlRpc::XmlRpcValue::iterator it;
  for (it = mappings_param.begin(); it != mappings_param.end(); ++it) {
    mappings.push_back(LoadMapping((*it).first, (*it).second));
  }
  
  return mappings;
}
  
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "voodoo_driver", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Get serial port mapping from ROS parameter
  std::vector<SerialMapping> mappings = LoadMappings(nh_private, "mapping");

  // Open all relevant serial ports
  std::map<std::string, int> ports;
  BOOST_FOREACH(SerialMapping mapping, mappings)
  {
    int fd = open(mapping.port_name.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
      ROS_WARN("Error %d opening %s: %s",
               errno, mapping.port_name.c_str(), strerror(errno));
    } else {
      ports[mapping.port_name] = fd;
    }

    SetSerialAttribs(fd, B115200, 0); // set speed to 115,200 bps, 8n1
    SetBlocking(fd, true);            // set blocking IO 
  }

  while(ros::ok()) {

    // Read from every analog input
    //    write(fd, "hello!\n", 7);      // send 7 character greeting
    //char buf [100];
    //int n = read (fd, buf, sizeof buf);  // read up to 100 characters if ready to read

    // Use mapping to convert to joint states
    

    // Publish joint state message
    
  }
}
