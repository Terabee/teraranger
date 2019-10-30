#include <teraranger/teraranger_evo.h>

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
  private_node_handle_.param("frame_id", frame_id_, std::string("base_range"));

  //Publishers
  range_publisher_ = nh_.advertise<sensor_msgs::Range>("teraranger_evo", 2);

  // Serial Port init
  serial_port_.setPort(portname_);
  serial_port_.setBaudrate(SERIAL_SPEED);
  serial_port_.setParity(serial::parity_none);
  serial_port_.setStopbits(serial::stopbits_one);
  serial_port_.setBytesize(serial::eightbits);
  serial::Timeout to = serial::Timeout::simpleTimeout(SERIAL_TIMEOUT_MS);
  serial_port_.setTimeout(to);

  serial_port_.open();

  // Connect serial port
  if(!serial_port_.isOpen())
  {
    ROS_ERROR("Could not open : %s ", portname_.c_str());
    ros::shutdown();
    return;
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
  range_msg.field_of_view = field_of_view;

//  private_node_handle_.deleteParam("sensor_type");
  private_node_handle_.getParam("sensor_type", sensor_type_);
  ROS_INFO("[%s] sensor type: %s", ros::this_node::getName().c_str(), sensor_type_.c_str());

  if (sensor_type_ == "Evo_60m")
  {
    range_msg.max_range = EVO_60M_MAX;
    range_msg.min_range = EVO_60M_MIN;
  }

  else if (sensor_type_ == "Evo_600Hz")
  {
    range_msg.max_range = EVO_600HZ_MAX;
    range_msg.min_range = EVO_600HZ_MIN;
  }

  else if (sensor_type_ == "Evo_3m")
  {
    range_msg.max_range = EVO_3M_MAX;
    range_msg.min_range = EVO_3M_MIN;
  }

  else if (!private_node_handle_.hasParam("sensor_type"))
  {
   ROS_INFO("No evo type set, Evo 60m by default");
   private_node_handle_.param("sensor_type", sensor_type_, std::string("Evo_60m"));
   range_msg.max_range = EVO_60M_MAX;
   range_msg.min_range = EVO_60M_MIN;
  }
  else
  {
   ROS_INFO("Unknow Evo type, Evo 60m by default");
   private_node_handle_.param("sensor_type", sensor_type_, std::string("Evo_60m"));
   range_msg.max_range = EVO_60M_MAX;
   range_msg.min_range = EVO_60M_MIN;
  }

  range_msg.header.frame_id = frame_id_;
  range_msg.radiation_type = sensor_msgs::Range::INFRARED;
}
TerarangerEvo::~TerarangerEvo() {}

void TerarangerEvo::setMode(const char *c, int length)
{
  if(!serial_port_.write((uint8_t*)c, length))
  {
    ROS_ERROR("Timeout or error while writing serial");
  }
  serial_port_.flushOutput();
}

void TerarangerEvo::serialDataCallback(uint8_t single_character)
{
  static uint8_t input_buffer[BUFFER_SIZE];
  static int buffer_ctr = 0;
  static int seq_ctr = 0;

  if (single_character == 'T' && buffer_ctr == 0)
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
      int16_t range = input_buffer[1] << 8;
      range |= input_buffer[2];

      float final_range;
      float float_range = range * VALUE_TO_METER_FACTOR;

      if(range == TOO_CLOSE_VALUE)// Too close, 255 is for short range
      {
        final_range = -std::numeric_limits<float>::infinity();
      }
      else if(range == OUT_OF_RANGE_VALUE)// Out of range
      {
        final_range = std::numeric_limits<float>::infinity();
      }
      else if(range == INVALID_MEASURE)// Cannot measure
      {
        final_range = std::numeric_limits<float>::quiet_NaN();
      }
      // Enforcing min and max range
      else if(float_range > range_msg.max_range)
      {
        final_range = std::numeric_limits<float>::infinity();
      }
      else if(float_range < range_msg.min_range)
      {
        final_range = -std::numeric_limits<float>::infinity();
      }
      else
      {
        final_range = float_range;
      }

      range_msg.header.stamp = ros::Time::now();
      range_msg.header.seq = seq_ctr++;
      range_msg.range = final_range;
      range_msg.header.stamp = ros::Time::now();
      range_publisher_.publish(range_msg);
    }
    else
    {
      ROS_DEBUG("[%s] crc missmatch", ros::this_node::getName().c_str());
    }
  }
  buffer_ctr = 0;
  bzero(&input_buffer, BUFFER_SIZE);
}

void TerarangerEvo::spin()
{
  static uint8_t buffer[1];
  while(ros::ok())
  {
    if(serial_port_.read(buffer, 1))
    {
      serialDataCallback(buffer[0]);
    }
    else
    {
      ROS_ERROR("Timeout or error while reading serial");
    }
    ros::spinOnce();
  }
}

} // namespace teraranger

int main(int argc, char **argv)
{
  ros::init(argc, argv, "teraranger_evo");
  teraranger::TerarangerEvo evo;
  evo.spin();

  return 0;
}
