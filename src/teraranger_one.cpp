/****************************************************************************
 *
 * Copyright (C) 2014 Flavio Fontana & Luis Rodrigues. All rights reserved.
 * Author: Flavio Fontana <fly.fontana@gmail.com>
 * Author: Luis Rodrigues <luis.rodrigues@terabee.com>

 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in
 * the documentation and/or other materials provided with the
 * distribution.
 * 3. Neither the name TerarangerOne nor the names of its contributors may be
 * used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#include <teraranger/teraranger_one.h>

namespace teraranger
{

TerarangerOne::TerarangerOne()
{
  // Get paramters
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("portname", portname_, std::string("/dev/ttyUSB0"));

  // Publishers
  range_publisher_ = nh_.advertise<sensor_msgs::Range>("terarangerone", 1);

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
  dyn_param_server_callback_function_ = boost::bind(&TerarangerOne::dynParamCallback, this, _1, _2);
  dyn_param_server_.setCallback(dyn_param_server_callback_function_);
}

TerarangerOne::~TerarangerOne()
{
}

void TerarangerOne::serialDataCallback(uint8_t single_character)
{
  static uint8_t input_buffer[BUFFER_SIZE];
  static int buffer_ctr = 0;
  static int seq_ctr = 0;

  sensor_msgs::Range range_msg;
  range_msg.field_of_view = 0.0593;
  range_msg.max_range = 14.0;
  range_msg.min_range = 0.2;
  range_msg.header.frame_id = "base_range";
  range_msg.radiation_type = sensor_msgs::Range::INFRARED;

  if (single_character != 'T' && buffer_ctr < 4)
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
        range_publisher_.publish(range_msg);

        ROS_DEBUG("[%s] all good %.3f m", ros::this_node::getName().c_str(), range_msg.range);
      }
      else
      {
        ROS_DEBUG("[%s] crc missmatch", ros::this_node::getName().c_str());
      }
    }
    else
    {
      ROS_DEBUG("[%s] reveived T but did not expect it, reset buffer without evaluating data",
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
  bzero(&input_buffer, BUFFER_SIZE);

  // store T
  input_buffer[buffer_ctr++] = 'T';
}

void TerarangerOne::setMode(const char *c)
{
  if(!serial_port_.write((uint8_t*)c, CMD_BYTE_LENGTH))// 1 byte commands
  {
    ROS_ERROR("Timeout or error while writing serial");
  }
  serial_port_.flushOutput();
}

void TerarangerOne::dynParamCallback(const teraranger::TerarangerOneConfig &config, uint32_t level)
{
  if (config.Mode == teraranger::TerarangerOne_Fast)
  {
    setMode(FAST_MODE);
  }

  if (config.Mode == teraranger::TerarangerOne_Precise)
  {
    setMode(PRECISE_MODE);
  }

  if (config.Mode == teraranger::TerarangerOne_Outdoor)
  {
    setMode(OUTDOOR_MODE);
  }
}

void TerarangerOne::spin()
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
  ros::init(argc, argv, "teraranger_one");
  teraranger::TerarangerOne one;
  one.spin();

  return 0;
}
