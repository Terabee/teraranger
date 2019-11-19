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
  range_publisher_ = nh_.advertise<sensor_msgs::Range>("teraranger_evo_mini/range", 1);
  ranges_publisher_ = nh_.advertise<teraranger_array::RangeArray>("teraranger_evo_mini/ranges", 1);

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
  //setMode(BINARY_MODE, 4);
  current_max = EVO_MINI_MAX_RANGE_SINGLE_LONG;
  current_min = EVO_MINI_MIN_RANGE_SINGLE_LONG;

  // Dynamic reconfigure
  dyn_param_server_callback_function_ =
    boost::bind(&TerarangerEvoMini::dynParamCallback, this, _1, _2);
  dyn_param_server_.setCallback(dyn_param_server_callback_function_);

  current_fov = EVO_MINI_TWO_BY_TWO_FOV;
  initRangeArrayMessage(EVO_MINI_MAX_PIXEL_COUNT);

  // Initialize Range message
  range_msg.field_of_view = EVO_MINI_SINGLE_PIXEL_FOV;
  range_msg.max_range = current_max;
  range_msg.min_range = current_min;
  // set the right range frame depending of the namespace
  if (ns_ == "")
  {
    range_msg.header.frame_id = frame_id_;
  }
  else
  {
    range_msg.header.frame_id = ns_ + '_'+ frame_id_;
  }
  range_msg.radiation_type = sensor_msgs::Range::INFRARED;
  range_msg.range = 0.0;
}

TerarangerEvoMini::~TerarangerEvoMini() {}

void TerarangerEvoMini::initRangeArrayMessage(int nb_ranges)
{
  range_array_msg.ranges.clear();
  // Initialize RangeArray message
  for (size_t i=0; i < nb_ranges; i++)
  {
    sensor_msgs::Range range;
    range.field_of_view = current_fov;
    range.max_range = current_max;
    range.min_range = current_min;
    range.radiation_type = sensor_msgs::Range::INFRARED;
    range.range = 0.0;
    // set the right range frame depending of the namespace
    if (ns_ == "")
    {
     range.header.frame_id = frame_id_;
    }
    else
    {
     range.header.frame_id = ns_ + '_'+ frame_id_;
    }
    range_array_msg.ranges.push_back(range);
  }
}

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

  // Dismiss buffered data until ACK header is found
  bool got_header = false;
  while(!got_header){
    serial_port_.read(ack_buffer, 1);
    if(ack_buffer[0] == ACK_HEADER) got_header = true;
  }

  if(serial_port_.read(ack_buffer+1, ACK_LENGTH-1)) // Read 3 more chars and append them to the buffer after the header
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

  ROS_DEBUG("Buffer counter %d Current char in buffer %d", buffer_ctr, single_character);

  if (buffer_ctr == 0)
  {
    if (single_character == RANGE_FRAME_HEADER)
    {
      input_buffer[buffer_ctr++] = single_character;

      ROS_DEBUG("T detected");
    }
    return;
  }
  else if (buffer_ctr == RANGE_FRAME_LENGTH_SINGLE_PIXEL-1)
  {
    ROS_DEBUG("Enough chars for single-range frame");

    input_buffer[buffer_ctr++] = single_character;
    uint8_t crc = HelperLib::crc8(input_buffer, RANGE_FRAME_LENGTH_SINGLE_PIXEL-1);
    if(crc == input_buffer[RANGE_FRAME_LENGTH_SINGLE_PIXEL-1])
    {
      ROS_DEBUG("Valid single-range frame");
      processSingleRangeFrame(input_buffer, seq_ctr);
    }
    else
    {
      ROS_DEBUG("[%s] crc missmatch for single-range frame", ros::this_node::getName().c_str());
      return;
    }
  }
  else if (buffer_ctr == RANGE_FRAME_LENGTH_TWO_PIXEL-1)
  {
    ROS_DEBUG("Enough chars for two-range frame");

    input_buffer[buffer_ctr++] = single_character;
    uint8_t crc = HelperLib::crc8(input_buffer, RANGE_FRAME_LENGTH_TWO_PIXEL-1);
    if(crc == input_buffer[RANGE_FRAME_LENGTH_TWO_PIXEL-1])
    {
      ROS_DEBUG("Valid two-range frame");
      processMultiRangeFrame(input_buffer, seq_ctr, 2);
    }
    else
    {
      ROS_DEBUG("[%s] crc missmatch for two-range frame", ros::this_node::getName().c_str());
      return;
    }
  }
  else if (buffer_ctr == RANGE_FRAME_LENGTH_TWO_BY_TWO-1)
  {
    ROS_DEBUG("Enough chars for two-by-two-range frame");

    input_buffer[buffer_ctr++] = single_character;
    uint8_t crc = HelperLib::crc8(input_buffer, RANGE_FRAME_LENGTH_TWO_BY_TWO-1);
    if(crc == input_buffer[RANGE_FRAME_LENGTH_TWO_BY_TWO-1])
    {
      ROS_DEBUG("Valid two-by-two-range frame");
      processMultiRangeFrame(input_buffer, seq_ctr, 4);
    }
    else
    {
      ROS_DEBUG("[%s] crc missmatch for two-by-two-range frame", ros::this_node::getName().c_str());
    }
  }
  else if (buffer_ctr < RANGE_FRAME_LENGTH_TWO_BY_TWO-1)
  {
    input_buffer[buffer_ctr++] = single_character;

    ROS_DEBUG("Gathering chars");
    return;
  }
  buffer_ctr = 0;
  bzero(&input_buffer, BUFFER_SIZE);

  // Appending current char to hook next frame
  if (single_character == RANGE_FRAME_HEADER)
  {
    input_buffer[buffer_ctr++] = single_character;
  }
}

float TerarangerEvoMini::processRawRangeValue(uint16_t raw_range)
{
  float float_range = raw_range * VALUE_TO_METER_FACTOR;
  float final_range = 0;

  if(raw_range == TOO_CLOSE_VALUE)// Too close, 255 is for short raw_range
  {
    final_range = -std::numeric_limits<float>::infinity();
  }
  else if(raw_range == OUT_OF_RANGE_VALUE)// Out of range
  {
    final_range = std::numeric_limits<float>::infinity();
  }
  else if(raw_range == INVALID_MEASURE)// Cannot measure
  {
    final_range = std::numeric_limits<float>::quiet_NaN();
  }
  else
  {
    final_range = float_range;
  }
  return final_range;
}

void TerarangerEvoMini::processSingleRangeFrame(uint8_t * frame_buffer, int seq)
{
  uint16_t range = frame_buffer[1] << 8;
  range |= frame_buffer[2];

  float processed_range = processRawRangeValue(range);

  ROS_DEBUG("Raw range %d, processed range %f", range, processed_range);

  range_msg.header.seq = seq++;
  range_msg.range = processed_range;
  range_msg.header.stamp = ros::Time::now();
  range_msg.max_range = current_max;
  range_msg.min_range = current_min;

  range_publisher_.publish(range_msg);

  checkSubscribers(false);
}

void TerarangerEvoMini::processMultiRangeFrame(uint8_t * frame_buffer, int seq, int range_count)
{
  ros::Time timestamp = ros::Time::now();
  for(size_t i = 0; i < range_array_msg.ranges.size(); i++)
    {
      // Convert bytes to range
      // Doesn't go out of range because of fixed buffer size as long as the number of sensor is not above 8
      char c1 = frame_buffer[2 * i + 1];
      char c2 = frame_buffer[2 * i + 2];
      int16_t range = (c1 & 0x0FF) << 8;
      range |= (c2 & 0x0FF);

      float processed_range = processRawRangeValue(range);

      ROS_DEBUG("Range [%f]", processed_range);

      range_array_msg.ranges.at(i).header.stamp = timestamp;
      range_array_msg.ranges.at(i).header.seq = seq++;
      range_array_msg.ranges.at(i).range = processed_range;
      range_array_msg.ranges.at(i).max_range = current_max;
      range_array_msg.ranges.at(i).min_range = current_min;
    }
  range_array_msg.header.seq = (int)seq/8;
  range_array_msg.header.stamp = timestamp;
  ranges_publisher_.publish(range_array_msg);

  checkSubscribers(true);
}

void TerarangerEvoMini::checkSubscribers(bool multi)
{
  int nb_sub_ranges = ranges_publisher_.getNumSubscribers();
  int nb_sub_range = range_publisher_.getNumSubscribers();
  if(multi)
  {
    if(nb_sub_ranges == 0 && nb_sub_range > 0)
    {
      ROS_WARN_THROTTLE(60, "Detected subscriber(s) only on \"range\" topic but the sensor is outputting multiple ranges");
    }
  }
  else
  {
    if(nb_sub_range == 0 && nb_sub_ranges > 0)
    {
      ROS_WARN_THROTTLE(60, "Detected subscriber(s) only on \"ranges\" topic but the sensor is outputting a single range");
    }
  }
}

void TerarangerEvoMini::reconfigurePixelMode(
  const teraranger::EvoMiniConfig &config)
{
  ROS_INFO("[%s] Reconfigure call: Pixel mode", ros::this_node::getName().c_str());
  if (config.Pixel_mode == teraranger::EvoMini_Single_pixel)
  {
    setMode(SINGLE_PIXEL_MODE, 4);
    current_pixel_mode = config.Pixel_mode;
    updateExtrema();
  }
  else if (config.Pixel_mode == teraranger::EvoMini_Two_pixel)
  {
    setMode(TWO_PIXEL_MODE, 4);
    current_fov = EVO_MINI_TWO_PIXEL_FOV;
    current_pixel_mode = config.Pixel_mode;
    initRangeArrayMessage(2);
    updateExtrema();
  }
  else if (config.Pixel_mode == teraranger::EvoMini_Two_by_two_pixel)
  {
    setMode(TWO_BY_TWO_PIXEL_MODE, 4);
    current_fov = EVO_MINI_TWO_BY_TWO_FOV;
    current_pixel_mode = config.Pixel_mode;
    initRangeArrayMessage(4);
    updateExtrema();
  }
  else ROS_ERROR("[%s] Invalid reconfigure option", ros::this_node::getName().c_str());
}

void TerarangerEvoMini::reconfigureRangeMode(
  const teraranger::EvoMiniConfig &config)
{
  ROS_INFO("[%s] Reconfigure call: Range mode", ros::this_node::getName().c_str());
  if (config.Range_mode == teraranger::EvoMini_Short)
  {
    setMode(SHORT_RANGE_MODE, 4);
    current_range_mode = config.Range_mode;
    updateExtrema();
  }
  else if (config.Range_mode == teraranger::EvoMini_Long)
  {
    setMode(LONG_RANGE_MODE, 4);
    current_range_mode = config.Range_mode;
    updateExtrema();
  }
  else ROS_ERROR("[%s] Invalid reconfigure option", ros::this_node::getName().c_str());
}

void TerarangerEvoMini::updateExtrema()
{
  if(current_range_mode == teraranger::EvoMini_Long)
  {
    if (current_pixel_mode == teraranger::EvoMini_Single_pixel)
    {
      current_max = EVO_MINI_MAX_RANGE_SINGLE_LONG;
      current_min = EVO_MINI_MIN_RANGE_SINGLE_LONG;
    }
    else if (current_pixel_mode == teraranger::EvoMini_Two_pixel)
    {
      current_max = EVO_MINI_MAX_RANGE_TWO_PIXEL_LONG;
      current_min = EVO_MINI_MIN_RANGE_TWO_PIXEL_LONG;
    }
    else if (current_pixel_mode == teraranger::EvoMini_Two_by_two_pixel)
    {
      current_max = EVO_MINI_MAX_RANGE_TWO_BY_TWO_LONG;
      current_min = EVO_MINI_MIN_RANGE_TWO_BY_TWO_LONG;
    }
  }
  else if (current_range_mode == teraranger::EvoMini_Short)
  {
    if (current_pixel_mode == teraranger::EvoMini_Single_pixel)
    {
      current_max = EVO_MINI_MAX_RANGE_SINGLE_SHORT;
      current_min = EVO_MINI_MIN_RANGE_SINGLE_SHORT; 
    }
    else if (current_pixel_mode == teraranger::EvoMini_Two_pixel)
    {
      current_max = EVO_MINI_MAX_RANGE_TWO_PIXEL_SHORT;
      current_min = EVO_MINI_MIN_RANGE_TWO_PIXEL_SHORT;
    }
    else if (current_pixel_mode == teraranger::EvoMini_Two_by_two_pixel)
    {
      current_max = EVO_MINI_MAX_RANGE_TWO_BY_TWO_SHORT;
      current_min = EVO_MINI_MIN_RANGE_TWO_BY_TWO_SHORT;
    } 
  }
}

void TerarangerEvoMini::dynParamCallback(
    const teraranger::EvoMiniConfig &config, uint32_t level)
{
  switch(level)
  {
    case 0xffffffff:// Catching first reconfigure call
      ROS_INFO("[%s] Initial reconfigure call", ros::this_node::getName().c_str());
      reconfigurePixelMode(config);
      reconfigureRangeMode(config);
      break;
    case 0:// Set the mode dynamically
      reconfigurePixelMode(config);
      break;
    case 1:// Set the rate dynamically
      reconfigureRangeMode(config);
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
