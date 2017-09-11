#pragma once

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>

#include <boost/thread.hpp>
#include <boost/function.hpp>

#include <string>

namespace teraranger
{

class SerialPort
{
public:
  SerialPort();
  virtual ~SerialPort();

  bool connect(const std::string port);
  void disconnect();

  bool sendChar(const char c[], int len);

  void setSerialCallbackFunction(boost::function<void(uint8_t)> *f);
  void serialThread();

  int serial_port_fd_;
  boost::thread serial_thread_;
  bool serial_thread_should_exit_;

  boost::function<void(uint8_t)> *serial_callback_function;
};

} // namespace teraranger