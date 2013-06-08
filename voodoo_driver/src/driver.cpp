
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "simpleserial.h"
#include <string>
#include <boost/foreach.hpp>

const double DEFAULT_JOINT_MIN = -2.618; // -150 degrees
const double DEFAULT_JOINT_MAX =  2.618; // +150 degrees

/**
 * Helper function for linearly scaling a value within some input range
 * to a specified output range.  Linearly interpolates past the min and max.
 */
double remap(const double input,
             const double in_min,  const double in_max,
             const double out_min, const double out_max)
{
  const double in_range = (in_max - in_min);
  const double out_range = (out_max - out_min);
  
  return (((input - in_min) / in_range) * out_range) + out_min;
}

/**
 * Tuple class for holding a single IO mapping.
 */
class SerialEntry
{
 public:
  SerialEntry() : joint(""), min(0), max(0) {}

  SerialEntry(const SerialEntry &that)
  {
    joint = that.joint;
    min = that.min;
    max = that.max;
  }

  SerialEntry(XmlRpc::XmlRpcValue &map_entry) :
      min(DEFAULT_JOINT_MIN), max(DEFAULT_JOINT_MAX)
  {
    ROS_ASSERT(map_entry.getType() == XmlRpc::XmlRpcValue::TypeStruct);

    ROS_ASSERT(map_entry.hasMember("joint"));
    joint = (std::string)map_entry["joint"];

    if (map_entry.hasMember("min"))
      min = (double)map_entry["min"];

    if (map_entry.hasMember("max"))
      max = (double)map_entry["max"];
  }

  virtual ~SerialEntry() {}
  
  std::string joint;
  double min;
  double max;
};

/**
 * Tuple class for holding serial port names and mappings of IO channel
 * to joint names.
 */
class SerialMapping
{
 public:
  SerialMapping(const std::string name, XmlRpc::XmlRpcValue &map)
  {
    ROS_ASSERT(map.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    port_name = "/dev/" + name;
    
    XmlRpc::XmlRpcValue::iterator it;
    for (it = map.begin(); it != map.end(); ++it) {
      int joint_idx = atoi((*it).first.c_str());
      ROS_ASSERT(joint_idx >= 0);
      ROS_ASSERT(joint_idx < 14);
      joints[joint_idx] = SerialEntry((*it).second);
    }
  }

  virtual ~SerialMapping() {}
  
  std::string port_name;
  std::map<uint8_t, SerialEntry> joints;
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
 * Typedefs for file description mapping (so I can use BOOST_FOREACH)
 */
typedef std::map<std::string, int> port_map_t;
typedef std::map<uint8_t, SerialEntry> daq_map_t;

/**
 * Main entry point.
 */
int main(int argc, char *argv[])
{
  // Initialize ROS node
  ros::init(argc, argv, "voodoo_driver", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ros::Publisher pub_joint_state =
      nh.advertise<sensor_msgs::JointState>("joint_states", 2);

  // Get serial port mapping from ROS parameter
  std::vector<SerialMapping> mappings = LoadMappings(nh_private, "mapping");

  // Open all relevant serial ports
  port_map_t ports;
  BOOST_FOREACH(const SerialMapping &mapping, mappings)
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

  // Blink the LEDs of all connected devices and setup channels
  BOOST_FOREACH(const SerialMapping &mapping, mappings)
  {
    // Retrieve file descriptor
    int fd = ports[mapping.port_name];
    
    // Clear system buffer and blink LED
    tcflush(fd, TCIFLUSH);
    write(fd, "\x02\x28", 2);

    // Setup requested channels
    BOOST_FOREACH(const daq_map_t::value_type &entry, mapping.joints)
    {
      write(fd, "\x04\x42", 2); // Configure ADC
      write(fd, &(entry.first), 1); // Select ADC channel
      write(fd, "\x09", 1); // 9-bit ADC conversion (fastest)
    }
  }

  // Begin loop of reading joint values
  while(ros::ok()) {

    // Send out query from every analog input on every device
    BOOST_FOREACH(const SerialMapping &mapping, mappings)
    {
      // Retrieve file descriptor
      int fd = ports[mapping.port_name];
      
      // Igmore any previous data
      tcflush(fd, TCIFLUSH);

      // Send requests for all requested channels
      BOOST_FOREACH(const daq_map_t::value_type &entry, mapping.joints)
      {
        write(fd, "\x03\x50", 2); // Request single ADC conversion
        write(fd, &(entry.first), 1); // Select ADC channel
      }
    }
    
    // Read back responses into joint state commands
    sensor_msgs::JointState joint_state;
    joint_state.header.frame_id = "/robot";
    joint_state.header.stamp = ros::Time::now();
    
    BOOST_FOREACH(const SerialMapping &mapping, mappings)
    {
      // Retrieve file description
      int fd = ports[mapping.port_name];
    
      // Read responses for each requested channels
      BOOST_FOREACH(const daq_map_t::value_type &entry, mapping.joints)
      {
        uint16_t adc_value;
        if (read(fd, &adc_value, 2) == 2)
        {
          double joint_value = remap(adc_value,
                                     0, 512,
                                     entry.second.min, entry.second.max);
          
          joint_state.name.push_back(entry.second.joint);
          joint_state.position.push_back(joint_value);
        }
        else
        {
          break;
        }
      }
    }

    // Publish joint state message
    pub_joint_state.publish(joint_state);
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
