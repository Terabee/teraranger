#include <teraranger/teraranger_evo_mini.h>

namespace teraranger
{

TerarangerEvoMini::TerarangerEvoMini()
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
  range_publisher_ = nh_.advertise<sensor_msgs::Range>("teraranger_evo_mini", 2);

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
  setMode(BINARY_MODE, 4);

  // Dynamic reconfigure
  dyn_param_server_callback_function_ =
    boost::bind(&TerarangerEvoMini::dynParamCallback, this, _1, _2);
  dyn_param_server_.setCallback(dyn_param_server_callback_function_);

//   // Initialize range message
//   range_msg.field_of_view = field_of_view;

// //  private_node_handle_.deleteParam("sensor_type");
//   private_node_handle_.getParam("sensor_type", sensor_type_);
//   ROS_INFO("[%s] sensor type: %s", ros::this_node::getName().c_str(), sensor_type_.c_str());

//   if (sensor_type_ == "Evo_60m")
//   {
//     range_msg.max_range = EVO_60M_MAX;
//     range_msg.min_range = EVO_60M_MIN;
//   }

//   else if (sensor_type_ == "Evo_600Hz")
//   {
//     range_msg.max_range = EVO_600HZ_MAX;
//     range_msg.min_range = EVO_600HZ_MIN;
//   }

//   else if (sensor_type_ == "Evo_3m")
//   {
//     range_msg.max_range = EVO_3M_MAX;
//     range_msg.min_range = EVO_3M_MIN;
//   }

//   else if (!private_node_handle_.hasParam("sensor_type"))
//   {
//    ROS_INFO("No evo type set, Evo 60m by default");
//    private_node_handle_.param("sensor_type", sensor_type_, std::string("Evo_60m"));
//    range_msg.max_range = EVO_60M_MAX;
//    range_msg.min_range = EVO_60M_MIN;
//   }
//   else
//   {
//    ROS_INFO("Unknow Evo type, Evo 60m by default");
//    private_node_handle_.param("sensor_type", sensor_type_, std::string("Evo_60m"));
//    range_msg.max_range = EVO_60M_MAX;
//    range_msg.min_range = EVO_60M_MIN;
//   }

//   range_msg.header.frame_id = frame_id_;
//   range_msg.radiation_type = sensor_msgs::Range::INFRARED;
}

TerarangerEvoMini::~TerarangerEvoMini() {}

void TerarangerEvoMini::setMode(const char *c, int length)
{
  serial_port_.flushInput();
  if(!serial_port_.write((uint8_t*)c, length))
  {
    ROS_ERROR("[%s] Timeout or error while writing to serial", ros::this_node::getName().c_str());
  }
  serial_port_.flushOutput();

  uint8_t ack_buffer[ACK_LENGTH];
  bool status = 0;

  if(serial_port_.read(ack_buffer, ACK_LENGTH))
  {
    status = processAck(ack_buffer, (uint8_t*)c);
  }
  else
  {
    ROS_ERROR("[%s] Timeout or error while waiting for ACK", ros::this_node::getName().c_str());
  }

  if(status){
    ROS_INFO("[%s] Command successful", ros::this_node::getName().c_str());
  }
  else
  {
    ROS_ERROR("[%s] Command not applied and/or not recognized", ros::this_node::getName().c_str());
  }
}

bool TerarangerEvoMini::processAck(uint8_t* ack_buffer, const uint8_t* cmd)
{
  uint8_t crc = HelperLib::crc8(ack_buffer, ACK_LENGTH-1);

  ROS_INFO("Header char %c", ack_buffer[0]);
  ROS_INFO("Header char %c", ack_buffer[1]);
  ROS_INFO("Header char %c", ack_buffer[2]);
  ROS_INFO("Header char %c", ack_buffer[3]);

  if (crc == ack_buffer[ACK_LENGTH-1])// Check is ACK frame is ok
  {
    if(ack_buffer[0] == ACK_HEADER)
    {
      if((cmd[1] >> 4) == ack_buffer[1])// See if the ack is from the same register as the command
      {
        if (ack_buffer[2] == ACK_VALUE)
        {
          return true;
        }
        else if (ack_buffer[2] == NACK_VALUE)
        {
          ROS_ERROR("[%s] Command was not acknowledged", ros::this_node::getName().c_str());
          return false;
        }
        else
        {
          ROS_ERROR("[%s] Invalid acknowledgment value", ros::this_node::getName().c_str());
          return false;
        }
      }
      else
      {
        ROS_ERROR("[%s] Wrong ack register", ros::this_node::getName().c_str());
        return false;
      }
    }
    else
    {
      ROS_ERROR("[%s] Wrong ack header", ros::this_node::getName().c_str());
      return false;
    }
  }
  else
  {
    ROS_ERROR("[%s] ACK frame crc missmatch", ros::this_node::getName().c_str());
    return false;
  }
}

void TerarangerEvoMini::serialDataCallback(uint8_t single_character)
{
  static uint8_t input_buffer[BUFFER_SIZE];
  static int buffer_ctr = 0;
  static int seq_ctr = 0;

  if (single_character == RANGE_FRAME_HEADER && buffer_ctr == 0)
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
  if (buffer_ctr == RANGE_FRAME_LENGTH_SINGLE-1)
  {
    input_buffer[buffer_ctr] = single_character;
    uint8_t crc = HelperLib::crc8(input_buffer, RANGE_FRAME_LENGTH_SINGLE-1);
    if(crc == input_buffer[RANGE_FRAME_LENGTH_SINGLE-1])
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

void TerarangerEvoMini::reconfigure_pixel_mode(
  const teraranger::EvoMiniConfig &config)
{
  ROS_INFO("[%s] Reconfigure call: Pixel mode", ros::this_node::getName().c_str());
  if (config.Pixel_mode == teraranger::EvoMini_Single)
  {
    setMode(SINGLE_RANGE_MODE, 4);
  }
  else if (config.Pixel_mode == teraranger::EvoMini_Multi)
  {
    setMode(MULTI_RANGE_MODE, 4);
  }
  else ROS_ERROR("[%s] Invalid reconfigure option", ros::this_node::getName().c_str());
}

void TerarangerEvoMini::reconfigure_range_mode(
  const teraranger::EvoMiniConfig &config)
{
  ROS_INFO("[%s] Reconfigure call: Range mode", ros::this_node::getName().c_str());
  if (config.Pixel_mode == teraranger::EvoMini_Single)
  {
    setMode(SHORT_RANGE_MODE, 4);
  }
  else if (config.Pixel_mode == teraranger::EvoMini_Multi)
  {
    setMode(LONG_RANGE_MODE, 4);
  }
  else ROS_ERROR("[%s] Invalid reconfigure option", ros::this_node::getName().c_str());
}

void TerarangerEvoMini::dynParamCallback(
    const teraranger::EvoMiniConfig &config, uint32_t level)
{
  switch(level)
  {
    case 0xffffffff:// Catching first reconfigure call
      ROS_INFO("[%s] Initial reconfigure call", ros::this_node::getName().c_str());
      reconfigure_pixel_mode(config);
      reconfigure_range_mode(config);
      break;
    case 0:// Set the mode dynamically
      reconfigure_pixel_mode(config);
      break;
    case 1:// Set the rate dynamically
      reconfigure_range_mode(config);
      break;
    default:
      ROS_ERROR("[%s] Invalid reconfigure level : %d", ros::this_node::getName().c_str(), level);
      break;
  }
}

void TerarangerEvoMini::spin()
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
  ros::init(argc, argv, "teraranger_evo_mini");
  teraranger::TerarangerEvoMini evo_mini;
  evo_mini.spin();

  return 0;
}
