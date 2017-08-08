#include <teraranger/teraranger_evo.h>
#include <sensor_msgs/Range.h>
#include <teraranger/helper_lib.h>

namespace teraranger
{
TerarangerEvo::TerarangerEvo()
{
  // Get parameters and namespace
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("portname", portname_,
                              std::string("/dev/ttyACM0"));
  ns_ = ros::this_node::getNamespace();
  ns_ = ros::names::clean(ns_);
  ROS_INFO("node namespace: [%s]", ns_.c_str());

  //Publishers
  range_publisher_ = nh_.advertise<sensor_msgs::Range>("teraranger_evo", 2);

  // Serial Port init
  serial_port_ = new SerialPort();
  if (!serial_port_->connect(portname_))
  {
      ROS_ERROR("Could not open : %s ", portname_.c_str());
      ros::shutdown();
      return;
  }
  else
  {
      serial_data_callback_function_ =
          boost::bind(&TerarangerEvo::serialDataCallback, this, _1);
      serial_port_->setSerialCallbackFunction(&serial_data_callback_function_);
  }

  // Output loaded parameters to console for double checking
  ROS_INFO("[%s] is up and running with the following parameters:",
              ros::this_node::getName().c_str());
  ROS_INFO("[%s] portname: %s", ros::this_node::getName().c_str(),
              portname_.c_str());

  // Set binary mode
  setMode(ENABLE_CMD, 5);
  setMode(BINARY_MODE, 4);
  
  // Initialize range message
  range.field_of_view = field_of_view;
  range.max_range = max_range;
  range.min_range = min_range;
  range.radiation_type = sensor_msgs::Range::INFRARED;
  range.range = -1.0;
}
TerarangerEvo::~TerarangerEvo() {}

void TerarangerEvo::setMode(const char *c, int length)
{
  serial_port_->sendChar(c, length);
}

void TerarangerEvo::serialDataCallback(uint8_t single_character)
{
  static uint8_t input_buffer[BUFFER_SIZE];
  static int buffer_ctr = 0;
  
  if (single_character =='T' && buffer_ctr == 0)
  {
    input_buffer[buffer_ctr] = single_character;
    buffer_ctr++;
    return;
  }
  else if (buffer_ctr >= 1 && buffer_ctr < BUFFER_SIZE-1)
  {
    input_buffer[buffer_ctr] = single_character;
    buffer_ctr++;
    return;
  }
  if (buffer_ctr == BUFFER_SIZE-1)
  {
    input_buffer[buffer_ctr] = single_character;
    uint8_t crc = HelperLib::crc8(input_buffer, BUFFER_SIZE-1);
    if(crc == input_buffer[BUFFER_SIZE-1])
    {
      uint8_t c1 = input_buffer[1];
      uint8_t c2 = input_buffer[2];
      float f_range = HelperLib::two_chars_to_float(c1, c2)*0.001;

      if (f_range < min_range)
      {
        f_range = -std::numeric_limits<float>::infinity();
      }
      else if(f_range > max_range)
      {
        f_range = std::numeric_limits<float>::infinity();
      }
      
      range.range = f_range;
      range.header.stamp = ros::Time::now();
      range_publisher_.publish(range);
    
    }
  }
  buffer_ctr = 0;
  bzero(&input_buffer, BUFFER_SIZE);
}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "teraranger_evo");
  teraranger::TerarangerEvo node;
  ros::spin();

  return 0;
}