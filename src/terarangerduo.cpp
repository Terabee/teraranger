/****************************************************************************
 *
 * Copyright (C) 2014 Flavio Fontana & Luis Rodrigues. All rights reserved.
 * Author: Flavio Fontana <fly.fontana@gmail.com>
 * Author: Luis Rodrigues <luis.rodrigues@terabee.com>
 * Author: Ehsan Asadi    <ehsan.asadi@gmail.com>

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
 * 3. Neither the name TerarangerDuo nor the names of its contributors may be
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

#include <string>

#include "terarangerduo/terarangerduo.h"

namespace terarangerduo
{

TerarangerDuo::TerarangerDuo()
{    
  // Get paramters
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("portname", portname_, std::string("/dev/ttyUSB0"));
  private_node_handle_.param("topicname", topicname_, std::string("teraranger/timeofflight"));
  private_node_handle_.param("topicnamei", topicname_i_, std::string("teraranger/ultrasound"));

  // Publishers
  range_publisher_ = nh_.advertise<sensor_msgs::Range>(topicname_, 1);
  range_publisher_i_ = nh_.advertise<sensor_msgs::Range>(topicname_i_, 1);
  
  // Create serial port
  serial_port_ = new SerialPort();

  // Set callback function for for the serial port
  serial_data_callback_function_duo_ = boost::bind(&TerarangerDuo::serialDataCallbackDuo, this, _1);
  serial_port_->setSerialCallbackFunction(&serial_data_callback_function_duo_);

  // Connect serial port
  if (!serial_port_->connect(portname_))
  {
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
}

TerarangerDuo::~TerarangerDuo()
{
}

uint8_t TerarangerDuo::crc8(uint8_t *p, uint8_t len)
{
  uint16_t i;
  uint16_t crc = 0x0;

  while (len--)
  {
    i = (crc ^ *p++) & 0xFF;
    crc = (crc_table[i] ^ (crc << 8)) & 0xFF;
  }
  return crc & 0xFF;
}

void TerarangerDuo::serialDataCallbackDuo(uint8_t single_character)
{
  static uint8_t input_buffer[BUFFER_SIZE_DUO];
  static int buffer_ctr = 0;
  static int seq_ctr = 0;
    
  sensor_msgs::Range range_trone_msg;
  range_trone_msg.field_of_view = 0.0593;
  range_trone_msg.max_range = 14.0;
  range_trone_msg.min_range = 0.2;
  range_trone_msg.radiation_type = sensor_msgs::Range::INFRARED;

  sensor_msgs::Range range_sonar_msg;
  range_sonar_msg.field_of_view = 0.0872;
  range_sonar_msg.max_range = 7.65;    
  range_sonar_msg.min_range = 0.05;
  range_sonar_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
 
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
      int16_t crc = crc8(input_buffer, 3);
 
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

        if (range <= 14000 && range >= 200)
        {
          range_trone_msg.header.stamp = ros::Time::now();
          range_trone_msg.header.seq = seq_ctr++;
          range_trone_msg.range = range * 0.001; // convert to m
          range_publisher_.publish(range_trone_msg);        
        }
        ROS_DEBUG("[%s] all good %.3f m", ros::this_node::getName().c_str(), range_trone_msg.range);
      }
    }
    else if (buffer_ctr == 7)
    {
      // end of feed, calculate
      int16_t crc = crc8(input_buffer, 6);

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

        if (range_trone <= 14000 && range_trone >= 200)
        {
          range_trone_msg.header.stamp = ros::Time::now();
          range_trone_msg.header.seq = seq_ctr++;
          range_trone_msg.header.frame_id = "base_link";
          range_trone_msg.range = range_trone * 0.001; // convert to m
          range_publisher_.publish(range_trone_msg);                                  
        }
        ROS_DEBUG("[%s] all good %.3f m", ros::this_node::getName().c_str(), range_trone_msg.range);

        if (range_sonar <= 7650 && range_sonar >= 50)
        {
          range_sonar_msg.header.stamp = ros::Time::now();
          range_sonar_msg.header.seq = seq_ctr;
          range_sonar_msg.header.frame_id = "base_link";
          range_sonar_msg.range = range_sonar * 0.001 ; // convert to m
          range_publisher_i_.publish(range_sonar_msg);                                  
        }
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

void TerarangerDuo::setMode(char c)
{
  serial_port_->sendChar(c);
}

void TerarangerDuo::dynParamCallback(const terarangerduo::TerarangerDuoConfig &config, uint32_t level)
{
  if (config.Mode == terarangerduo::TerarangerDuo_Fast)
  {
    setMode(FAST_MODE);
  }

  if (config.Mode == terarangerduo::TerarangerDuo_Precise)
  {
    setMode(PRECISE_MODE);
  }

  if (config.Mode == terarangerduo::TerarangerDuo_Outdoor)
  {
    setMode(OUTDOOR_MODE);
  }

}

} // namespace terarangerduo

int main(int argc, char **argv)
{

  ros::init(argc, argv, "terarangerduo");
  terarangerduo::TerarangerDuo tera_bee;
  ros::spin();
 
  return 0;
}



