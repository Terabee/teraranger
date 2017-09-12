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
#ifndef TERARANGERONE_TERARANGER_H_
#define TERARANGERONE_TERARANGER_H_

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Char.h>
#include <dynamic_reconfigure/server.h>
#include <string>
#include <serial/serial.h>
#include <teraranger/TerarangerOneConfig.h>
#include <teraranger/helper_lib.h>
#include <limits>

#define OUT_OF_RANGE_VALUE 0
#define TOO_CLOSE_VALUE 200
#define VALUE_TO_METER_FACTOR 0.001

#define BUFFER_SIZE 4
#define SERIAL_SPEED 115200
#define SERIAL_TIMEOUT_MS 1000
#define CMD_BYTE_LENGTH 1

namespace teraranger
{

static const char PRECISE_MODE[CMD_BYTE_LENGTH] = {'P'};
static const char FAST_MODE[CMD_BYTE_LENGTH] = {'F'};
static const char OUTDOOR_MODE[CMD_BYTE_LENGTH] = {'O'};

static const char BINARY_MODE[CMD_BYTE_LENGTH] = {'B'};
static const char TEXT_MODE[CMD_BYTE_LENGTH] = {'T'};

class TerarangerOne
{
public:
  TerarangerOne();
  virtual ~TerarangerOne();

  void serialDataCallback(uint8_t data);

  void dynParamCallback(const teraranger::TerarangerOneConfig &config, uint32_t level);

  bool loadParameters();
  void setMode(const char *c);

  ros::NodeHandle nh_;
  ros::Publisher range_publisher_;

  dynamic_reconfigure::Server<teraranger::TerarangerOneConfig> dyn_param_server_;
  dynamic_reconfigure::Server<teraranger::TerarangerOneConfig>::CallbackType dyn_param_server_callback_function_;

  serial::Serial serial_port_;
  boost::function<void(uint8_t)> serial_data_callback_function_;

  std::string portname_;
  void spin();
};

} // namespace teraranger

#endif  // TERARANGERONE_TERARANGER_H_
