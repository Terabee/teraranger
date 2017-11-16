#pragma once

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
  std::string frame_id_;
  void spin();
};

} // namespace teraranger
