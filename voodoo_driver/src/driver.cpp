
#include "simpleserial.h"
#include <string>
#include <boost/foreach.hpp>

/**
 * Tuple class for holding a single IO mapping.
 */
class SerialEntry
{
 public:
  SerialEntry() : joint(""), offset(0) {}
  SerialEntry(const SerialEntry &that) {
    joint = that.joint;
    offset = that.offset;
  }
  SerialEntry(XmlRpc::XmlRpcValue &map_entry) : offset(0) {
    ROS_ASSERT(map_entry.getType() == XmlRpc::XmlRpcValue::TypeStruct);

    ROS_ASSERT(map_entry.hasMember("joint"));
    joint = (std::string)map_entry["joint"];

    if (map_entry.hasMember("offset"))
      offset = (double)map_entry["offset"];
  }
  virtual ~SerialEntry() {}
  
  std::string joint;
  double offset;
};

/**
 * Tuple class for holding serial port names and mappings of IO channel
 * to joint names.
 */
class SerialMapping
{
 public:
  SerialMapping(const std::string name, XmlRpc::XmlRpcValue &map) {
    ROS_ASSERT(map.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    port_name = "/dev/" + name;
    
    XmlRpc::XmlRpcValue::iterator it;
    for (it = map.begin(); it != map.end(); ++it) {
      joints[atoi((*it).first.c_str())] = SerialEntry((*it).second);
    }
  }
  virtual ~SerialMapping() {}
  
  std::string port_name;
  std::map<int, SerialEntry> joints;
};

/**
 * Helper function to load multiple DAQ configurations from ROS param.
 */
std::vector<SerialMapping> LoadMappings(const ros::NodeHandle &nh,
                                        const std::string &param_name)
{
  std::vector<SerialMapping> mappings;
  
  XmlRpc::XmlRpcValue mappings_param;
  nh.getParam(param_name, mappings_param);

  ROS_ASSERT(mappings_param.getType() == XmlRpc::XmlRpcValue::TypeStruct);

  XmlRpc::XmlRpcValue::iterator it;
  for (it = mappings_param.begin(); it != mappings_param.end(); ++it) {
    mappings.push_back(SerialMapping((*it).first, (*it).second));
  }
  
  return mappings;
}

/**
 * Typedef for file description mapping (so I can use BOOST_FOREACH)
 */
typedef std::map<std::string, int> port_map_t;

/**
 * Main entry point.
 */
int main(int argc, char *argv[])
{
  // Initialize ROS node
  ros::init(argc, argv, "voodoo_driver", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Get serial port mapping from ROS parameter
  std::vector<SerialMapping> mappings = LoadMappings(nh_private, "mapping");

  // Open all relevant serial ports
  port_map_t ports;
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

  // Close all relevant serial ports
  BOOST_FOREACH(const port_map_t::value_type &entry, ports)
  {
    if (close(entry.second) != 0) {
      ROS_WARN("Error %d closing %s: %s",
               errno, entry.first.c_str(), strerror(errno));
    }
  }
}
