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

#include <string>

#include "terarangerone/serial_port.h"
#include "terarangerone/terarangerone.h"

namespace terarangerone
{

SerialPort::SerialPort() :
    serial_callback_function()
{
  serial_thread_should_exit_ = false;
  serial_port_fd_ = 0;
}

SerialPort::~SerialPort()
{
  disconnect();
}

bool SerialPort::connect(const std::string port)
{
  // Open serial port
  // O_RDWR - Read and write
  // O_NOCTTY - Ignore special chars like CTRL-C
  serial_port_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
  if (serial_port_fd_ == -1)
  {
    ROS_ERROR("%s could not open serial port %s", ros::this_node::getName().c_str(), port.c_str());
    return false;
  }
  else
  {
    fcntl(serial_port_fd_, F_SETFL, 0);
  }

  struct termios newtio;
  bzero(&newtio, sizeof(newtio)); // clear struct for new port settings

  cfsetispeed(&newtio, B115200); // Input port speed
  cfsetospeed(&newtio, B115200); // Output port speed

  newtio.c_cflag &= ~PARENB; // no parity bit
  newtio.c_cflag &= ~CSTOPB; // 1 stop bit
  newtio.c_cflag &= ~CSIZE; // Only one stop bit
  newtio.c_cflag |= CS8; // 8 bit word

  newtio.c_iflag = 0; // Raw output since no parity checking is done
  newtio.c_oflag = 0; // Raw output
  newtio.c_lflag = 0; // Raw input is unprocessed

  tcflush(serial_port_fd_, TCIFLUSH);
  tcsetattr(serial_port_fd_, TCSANOW, &newtio);

  serial_thread_ = boost::thread(&SerialPort::serialThread, this);
  return true;
}

void SerialPort::disconnect()
{
  serial_thread_should_exit_ = true;
  // TODO(lfr) wait for thread to finish
  close(serial_port_fd_);
}

bool SerialPort::sendChar(const char c)
{
  return write(serial_port_fd_, (const void*)&c, 1);
}

void SerialPort::setSerialCallbackFunction(boost::function<void(uint8_t)> * f)
{
  serial_callback_function = f;
}

void SerialPort::serialThread()
{
  uint8_t single_character;
  // Non read
  while (!serial_thread_should_exit_ && ros::ok())
  {
    if (read(serial_port_fd_, &single_character, 1))
    {
      (*serial_callback_function)(single_character);
    }
    ros::Duration(0.0001).sleep();
  }
  return;
}

} // namespace terarangerone
