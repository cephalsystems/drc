
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>

#include "simpleserial.h"
#include <string>
#include <boost/foreach.hpp>

const double DEFAULT_JOINT_MIN = -2.618; // -150 degrees
const double DEFAULT_JOINT_MAX =  2.618; // +150 degrees

/**
 * Helper function for linearly scaling a value within some input range
 * to a specified output range.  Linearly interpolates past the min and max.
 */
inline double remap(double input,
                    double in_min,  double in_max,
                    double out_min, double out_max)
{
  const double in_range = (in_max - in_min);
  const double out_range = (out_max - out_min);
  
  return (((input - in_min) / in_range) * out_range) + out_min;
}

/**
 * Helper function to clip a value between a specified range.
 */
inline double clip(double input, double min, double max)
{
  return std::min(std::max(input, min), max);
}

/**
 * Tuple class for holding a single IO mapping.
 */
class SerialEntry
{
 public:
  SerialEntry() : joint(""), min(0), max(0), offset(0) {}

  SerialEntry(const SerialEntry &that)
  {
    joint = that.joint;
    min = that.min;
    max = that.max;
    offset = that.offset;
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

    if (map_entry.hasMember("offset"))
      offset = (double)map_entry["offset"];
  }

  virtual ~SerialEntry() {}
  
  std::string joint;
  double min;
  double max;
  double offset;
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
typedef std::map<std::string, boost::shared_ptr<urdf::Joint> > joint_map_t;

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

  // Retrieve current robot model
  std::string robot_description;
  if (!nh.getParam("robot_description", robot_description)) 
  {
    ROS_ERROR("No robot description parameter found!");
    return -1;
  }

  // Load robot model from parameter string
  urdf::Model urdf;
  if (!urdf.initString(robot_description)) 
  {
    ROS_ERROR("Failed to parse the URDF");
    return -1;
  }

  // Get serial port mapping from ROS parameter
  std::vector<SerialMapping> mappings = LoadMappings(nh_private, "mapping");

  // Open all relevant serial ports
  port_map_t ports;
  BOOST_FOREACH(const SerialMapping &mapping, mappings)
  {
    // Open serial port file descriptor
    int fd = open(mapping.port_name.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
      ROS_WARN("Error %d opening %s: %s",
               errno, mapping.port_name.c_str(), strerror(errno));
      continue;
    }
    
    // Set speed to 115,200 bps, 8n1, no flow ctl
    if (SetSerialAttribs(fd, B115200, 0) < 0) 
    {
      ROS_WARN("Error setting baud rate on port %s.", mapping.port_name.c_str());
      continue;
    }

    // Set 'non'-blocking IO (timeout in 500ms)
    if (SetBlocking(fd, false) < 0)
    {
      ROS_WARN("Error setting timeout on port %s.", mapping.port_name.c_str());
      continue;
    }

    // Store the valid file descriptor into mapping
    ports[mapping.port_name] = fd;
  }

  // Blink the LEDs of all connected devices and setup channels
  BOOST_FOREACH(const SerialMapping &mapping, mappings)
  {
    // Retrieve file descriptor
    port_map_t::iterator fd_iter = ports.find(mapping.port_name);
    if (fd_iter == ports.end())
      continue;
    int fd = (*fd_iter).second;
    
    // Clear system buffer
    tcflush(fd, TCIFLUSH);
    
    // Turn on power to pullup pin RB7
    if (write(fd, "\x05\x35\x12\x00\x01", 5) != 5) {
      ROS_ERROR("Error %d writing to %s: %s",
                errno, mapping.port_name.c_str(), strerror(errno));
    }

    // Turn on power to pullup pin RB6
    if (write(fd, "\x05\x35\x13\x00\x01", 5) != 5) {
      ROS_ERROR("Error %d writing to %s: %s",
                errno, mapping.port_name.c_str(), strerror(errno));
    }

    // Turn on power to pullup pin RA4
    if (write(fd, "\x05\x35\x0E\x00\x01", 5) != 5) {
      ROS_ERROR("Error %d writing to %s: %s",
                errno, mapping.port_name.c_str(), strerror(errno));
    }
    
    // Blink LED to indicate completed configuration
    if (write(fd, "\x02\x28", 2) != 2) {
      ROS_ERROR("Error %d writing to %s: %s",
                errno, mapping.port_name.c_str(), strerror(errno));
    }
  }

  // Set up fixed publish rate
  int rate;
  nh_private.param("rate", rate, 100);
  ros::Rate r(rate);
  
  // Begin loop of reading joint values
  ROS_INFO("Starting voodoo driver.");
  while(ros::ok()) {

    // Send out query from every analog input on every device
    BOOST_FOREACH(const SerialMapping &mapping, mappings)
    {
      // Retrieve file descriptor
      port_map_t::iterator fd_iter = ports.find(mapping.port_name);
      if (fd_iter == ports.end())
        continue;
      int fd = (*fd_iter).second;
      
      // Ignore any previous data
      tcflush(fd, TCIFLUSH);

      // Send requests for all requested channels
      BOOST_FOREACH(const daq_map_t::value_type &entry, mapping.joints)
      {
        // Request single ADC conversion
        if (write(fd, "\x03\x50", 2) != 2) {
          ROS_ERROR("Error %d writing to %s: %s",
                    errno, mapping.port_name.c_str(), strerror(errno));
        }

        // Select ADC channel
        if (write(fd, &(entry.first), 1) != 1) {
          ROS_ERROR("Error %d writing to %s: %s",
                    errno, mapping.port_name.c_str(), strerror(errno));
        }
      }
    }
    
    // Read back responses into joint state commands
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    
    // Fill in joint states from serial mapping
    BOOST_FOREACH(const SerialMapping &mapping, mappings)
    {
      // Retrieve file descriptor
      port_map_t::iterator fd_iter = ports.find(mapping.port_name);
      if (fd_iter == ports.end())
        continue;
      int fd = (*fd_iter).second;
    
      // Read responses for each requested channels
      BOOST_FOREACH(const daq_map_t::value_type &entry, mapping.joints)
      {
        uint16_t adc_value;
        if (read(fd, ((char*)&adc_value), 1) > 0 && 
            read(fd, ((char*)&adc_value) + 1, 1) > 0)
        {
          // Scale the value to the configured range
          double raw_joint_value = remap(adc_value,
                                         0, 1024,
                                         entry.second.min, entry.second.max);
          raw_joint_value += entry.second.offset;

          // Clip the value according to the URDF model
          boost::shared_ptr<urdf::JointLimits> limits =
              urdf.getJoint(entry.second.joint)->limits;
          double joint_value = clip(raw_joint_value, limits->lower, limits->upper);

          // Load the joint position into the joint state
          joint_state.name.push_back(entry.second.joint);
          joint_state.position.push_back(joint_value);

          //ROS_INFO("%s\t%u\t%f\t%f", entry.second.joint.c_str(),
          //         adc_value, raw_joint_value, joint_value);
        }
        else
        {
          ROS_WARN("Communication error to %s", mapping.port_name.c_str());
          break;
        }
      }
    }

    // Fill in remaining joints as zero-position
    BOOST_FOREACH(const joint_map_t::value_type &joint, urdf.joints_) 
    {
      if (std::find(joint_state.name.begin(), joint_state.name.end(), joint.first)
	  == joint_state.name.end()) {
	joint_state.name.push_back(joint.first);
	joint_state.position.push_back(0.0);
      } 
    }

    // Publish joint state message
    pub_joint_state.publish(joint_state);

    // Wait for next timestep
    ros::spinOnce();
    r.sleep();
  }
  ROS_INFO("Stopping voodoo driver.");

  // Close all relevant serial ports
  BOOST_FOREACH(const port_map_t::value_type &entry, ports)
  {
    if (close(entry.second) != 0) {
      ROS_WARN("Error %d closing %s: %s",
               errno, entry.first.c_str(), strerror(errno));
    }
  }
}
