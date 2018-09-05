#pragma once

#include <sensor_msgs/Range.h>
#include <serial/serial.h>
#include <ros/ros.h>
#include <teraranger/helper_lib.h>
#include <limits>

#define OUT_OF_RANGE_VALUE -1
#define TOO_CLOSE_VALUE 0
#define INVALID_MEASURE 1
#define VALUE_TO_METER_FACTOR 0.001

#define EVO_60M_MAX 60.0
#define EVO_60M_MIN 0.5
#define EVO_600HZ_MAX 8.0
#define EVO_600HZ_MIN 0.75
#define EVO_3M_MAX 3.0
#define EVO_3M_MIN 0.1

#define SERIAL_SPEED 115200
#define SERIAL_TIMEOUT_MS 1000

namespace teraranger
{
static const char ENABLE_CMD[5] = {(char)0x00, (char)0x52, (char)0x02, (char)0x01, (char)0xDF};
static const char TEXT_MODE[4] = {(char)0x00, (char)0x11, (char)0x01, (char)0x45};
static const char BINARY_MODE[4] = {(char)0x00, (char)0x11, (char)0x02, (char)0x4C};
static const uint8_t BUFFER_SIZE = 4;

class TerarangerEvo
{
  public:
    TerarangerEvo();
    virtual ~TerarangerEvo();

    void serialDataCallback(uint8_t data);

    bool loadParameters();
    void setMode(const char *c, int length);

    ros::NodeHandle nh_;
    ros::Publisher range_publisher_;

    serial::Serial serial_port_;
    boost::function<void(uint8_t)> serial_data_callback_function_;

    std::string portname_;
    std::string frame_id_;
    std::string ns_;
    std::string sensor_type_;


    void spin();

  private:
    const float field_of_view = 0.0349066f;
    const std::string frame_id = "base_range_";
    sensor_msgs::Range range_msg;
};
}
