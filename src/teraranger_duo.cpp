#include <teraranger/teraranger_duo.h>

namespace teraranger
{

TerarangerDuo::TerarangerDuo()
{
  // Get parameters
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("portname", portname_, std::string("/dev/ttyUSB0"));
  private_node_handle_.param("topicname", topicname_, std::string("teraranger_duo/infrared"));
  private_node_handle_.param("topicnamei", topicname_i_, std::string("teraranger_duo/ultrasound"));
  private_node_handle_.param("frame_id_ir", frame_id_ir_, std::string("base_range_ir"));
  private_node_handle_.param("frame_id_us", frame_id_us_, std::string("base_range_us"));

  // Publishers
  range_publisher_ = nh_.advertise<sensor_msgs::Range>(topicname_, 1);
  range_publisher_i_ = nh_.advertise<sensor_msgs::Range>(topicname_i_, 1);

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
  ROS_INFO("[%s] is up and running with the following parameters:", ros::this_node::getName().c_str());
  ROS_INFO("[%s] portname: %s", ros::this_node::getName().c_str(), portname_.c_str());

  // Set operation Mode
  setMode(BINARY_MODE);

  // Dynamic reconfigure
  dyn_param_server_callback_function_ = boost::bind(&TerarangerDuo::dynParamCallback, this, _1, _2);
  dyn_param_server_.setCallback(dyn_param_server_callback_function_);

  range_trone_msg.field_of_view = 0.0593;
  range_trone_msg.max_range = 14.0;
  range_trone_msg.min_range = 0.2;
  range_trone_msg.header.frame_id = frame_id_ir_;
  range_trone_msg.radiation_type = sensor_msgs::Range::INFRARED;

  range_sonar_msg.field_of_view = 0.0872;
  range_sonar_msg.max_range = 7.65;
  range_sonar_msg.min_range = 0.05;
  range_sonar_msg.header.frame_id = frame_id_us_;
  range_sonar_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
}

TerarangerDuo::~TerarangerDuo()
{
}

float TerarangerDuo::process_infrared_range(int16_t range)
{
  float final_range;
  float float_range = range * VALUE_TO_METER_FACTOR;

  if(range == TOO_CLOSE_VALUE)// Too close
  {
    final_range = -std::numeric_limits<float>::infinity();
  }
  else if(range == INFRARED_OUT_OF_RANGE)// Out of range
  {
    final_range = std::numeric_limits<float>::infinity();
  }
  // Enforcing min and max range
  else if(float_range > range_trone_msg.max_range)
  {
    final_range = std::numeric_limits<float>::infinity();
  }
  else if(float_range < range_trone_msg.min_range)
  {
    final_range = -std::numeric_limits<float>::infinity();
  }
  else
  {
    final_range = float_range;
  }
  return final_range;
}

float TerarangerDuo::process_ultrasound_range(int16_t range)
{
  float final_range;
  float float_range = range * VALUE_TO_METER_FACTOR;

  if(range == TOO_CLOSE_VALUE)// Too close
  {
    final_range = -std::numeric_limits<float>::infinity();
  }
  else if(range == ULTRASOUND_OUT_OF_RANGE)// Out of range
  {
    final_range = std::numeric_limits<float>::infinity();
  }
  // Enforcing min and max range
  else if(float_range > range_sonar_msg.max_range)
  {
    final_range = std::numeric_limits<float>::infinity();
  }
  else if(float_range < range_sonar_msg.min_range)
  {
    final_range = -std::numeric_limits<float>::infinity();
  }
  else
  {
    final_range = float_range;
  }
  return final_range;
}

void TerarangerDuo::serialDataCallbackDuo(uint8_t single_character)
{
  static uint8_t input_buffer[BUFFER_SIZE_DUO];
  static int buffer_ctr = 0;
  static int seq_ctr = 0;

  if (single_character != 'T' && buffer_ctr < 7)
  {
    // not begin of serial feed so add char to buffer
    input_buffer[buffer_ctr++] = single_character;
    return;
  }
  else if (single_character == 'T')
  {
    if (buffer_ctr == 4)
    {
      // end of feed, calculate
      int16_t crc = HelperLib::crc8(input_buffer, 3);

      if (crc == input_buffer[3])
      {
        // trduo_ = false;
        if (seq_ctr==1)
        {
          // Output loaded parameters to console for double checking
          ROS_WARN("expected TerarangerDuo but TerarangerOne is detected");
          ROS_INFO("[%s] is up and running with the following parameters:", ros::this_node::getName().c_str());
          ROS_INFO("[%s] portname: %s", ros::this_node::getName().c_str(), portname_.c_str());
          ROS_INFO("[%s] topicname: %s", ros::this_node::getName().c_str(), topicname_.c_str());
        }

        int16_t range = input_buffer[1] << 8;
        range |= input_buffer[2];

        range_trone_msg.header.stamp = ros::Time::now();
        range_trone_msg.header.seq = seq_ctr++;
        range_trone_msg.range = process_infrared_range(range); // convert to m
        range_publisher_.publish(range_trone_msg);
        ROS_DEBUG("[%s] all good %.3f m", ros::this_node::getName().c_str(), range_trone_msg.range);
      }
    }
    else if (buffer_ctr == 7)
    {
      // end of feed, calculate
      int16_t crc = HelperLib::crc8(input_buffer, 6);

      if (crc == input_buffer[6])
      {
        // trduo_ = true;
        if (seq_ctr==1)
        {
          // Output loaded parameters to console for double checking
          ROS_INFO("[%s] is up and running with the following parameters:", ros::this_node::getName().c_str());
          ROS_INFO("[%s] portname: %s", ros::this_node::getName().c_str(), portname_.c_str());
          ROS_INFO("[%s] topicname: %s", ros::this_node::getName().c_str(), topicname_.c_str());
          ROS_INFO("[%s] topicname: %s", ros::this_node::getName().c_str(), topicname_i_.c_str());
        }

        uint16_t range_trone = input_buffer[1] << 8;
        range_trone |= input_buffer[2];

        uint16_t range_sonar = input_buffer[4] << 8;
        range_sonar |= input_buffer[5];

        // Process infrared
        range_trone_msg.header.stamp = ros::Time::now();
        range_trone_msg.header.seq = seq_ctr++;
        range_trone_msg.range = process_infrared_range(range_trone); // convert to m
        range_publisher_.publish(range_trone_msg);
        ROS_DEBUG("[%s] all good %.3f m", ros::this_node::getName().c_str(), range_trone_msg.range);

        // Process ultrasound
        range_sonar_msg.header.stamp = ros::Time::now();
        range_sonar_msg.header.seq = seq_ctr;
        range_sonar_msg.range = process_ultrasound_range(range_sonar); // convert to m
        range_publisher_i_.publish(range_sonar_msg);
        ROS_DEBUG("[%s] all good %.3f m", ros::this_node::getName().c_str(), range_sonar_msg.range);
      }
      else
      {
        ROS_DEBUG("[%s] crc missmatch", ros::this_node::getName().c_str());
      }
    }
    else
    {
      ROS_DEBUG("[%s] received T but did not expect it, reset buffer without evaluating data",
               ros::this_node::getName().c_str());
    }
  }
  else
  {
    ROS_DEBUG("[%s] buffer_overflowed without receiving T, reset input_buffer", ros::this_node::getName().c_str());
  }
  // reset
  buffer_ctr = 0;

  // clear struct
  bzero(&input_buffer, BUFFER_SIZE_DUO);

  // store T
  input_buffer[buffer_ctr++] = 'T';
}

void TerarangerDuo::setMode(const char *c)
{
  if(!serial_port_.write((uint8_t*)c, CMD_BYTE_LENGTH))// 1 byte commands
  {
    ROS_ERROR("Timeout or error while writing serial");
  }
  serial_port_.flushOutput();
}

void TerarangerDuo::dynParamCallback(const teraranger::TerarangerDuoConfig &config, uint32_t level)
{
  if (config.Mode == teraranger::TerarangerDuo_Fast)
  {
    setMode(FAST_MODE);
  }

  if (config.Mode == teraranger::TerarangerDuo_Precise)
  {
    setMode(PRECISE_MODE);
  }

  if (config.Mode == teraranger::TerarangerDuo_Outdoor)
  {
    setMode(OUTDOOR_MODE);
  }

}

void TerarangerDuo::spin()
{
  static uint8_t buffer[1];
  while(ros::ok())
  {
    if(serial_port_.read(buffer, 1))
    {
      serialDataCallbackDuo(buffer[0]);
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

  ros::init(argc, argv, "teraranger_duo");
  teraranger::TerarangerDuo duo;
  duo.spin();

  return 0;
}
