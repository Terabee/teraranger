#pragma once

#include <sensor_msgs/Range.h>
#include <serial/serial.h>
#include <ros/ros.h>

#include <teraranger/helper_lib.h>
#include <dynamic_reconfigure/server.h>
#include <teraranger/EvoMiniConfig.h>
#include <teraranger_array/RangeArray.h>
#include <limits>

#define OUT_OF_RANGE_VALUE 0xFFFF
#define TOO_CLOSE_VALUE 0x0000
#define INVALID_MEASURE 0x0001
#define VALUE_TO_METER_FACTOR 0.001

#define EVO_MINI_MAX_RANGE_SINGLE_SHORT 1.35
#define EVO_MINI_MAX_RANGE_SINGLE_LONG 3.2
#define EVO_MINI_MAX_RANGE_MULTI_SHORT 1.35
#define EVO_MINI_MAX_RANGE_MULTI_LONG 1.65

#define EVO_MINI_MIN_RANGE_SINGLE_SHORT 0.04
#define EVO_MINI_MIN_RANGE_SINGLE_LONG 0.02
#define EVO_MINI_MIN_RANGE_MULTI_SHORT 0.04
#define EVO_MINI_MIN_RANGE_MULTI_LONG 0.04

#define SERIAL_SPEED 115200
#define SERIAL_TIMEOUT_MS 1000

#define ACK_HEADER 0x14
#define ACK_VALUE 0x00
#define NACK_VALUE 0xFF
#define ACK_LENGTH 4

#define BUFFER_SIZE 10
#define RANGE_FRAME_HEADER 'T'
#define RANGE_FRAME_LENGTH_SINGLE 4
#define RANGE_FRAME_LENGTH_MULTI 10

namespace teraranger
{
static const char ENABLE_CMD[5] = {(char)0x00, (char)0x52, (char)0x02, (char)0x01, (char)0xDF};

static const char TEXT_MODE[4] = {(char)0x00, (char)0x11, (char)0x01, (char)0x45};
static const char BINARY_MODE[4] = {(char)0x00, (char)0x11, (char)0x02, (char)0x4C};
static const char SINGLE_RANGE_MODE[4] = {(char)0x00, (char)0x21, (char)0x01, (char)0xBC};
static const char MULTI_RANGE_MODE[4] = {(char)0x00, (char)0x21, (char)0x02, (char)0xB5};
static const char LONG_RANGE_MODE[4] = {(char)0x00, (char)0x61, (char)0x03, (char)0xE9};
static const char SHORT_RANGE_MODE[4] = {(char)0x00, (char)0x61, (char)0x01, (char)0xE7};

class TerarangerEvoMini
{
  public:
    TerarangerEvoMini();
    virtual ~TerarangerEvoMini();

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

    void dynParamCallback(const teraranger::EvoMiniConfig &config, uint32_t level);
    dynamic_reconfigure::Server<teraranger::EvoMiniConfig> dyn_param_server_;
    dynamic_reconfigure::Server<teraranger::EvoMiniConfig>::CallbackType dyn_param_server_callback_function_;

    void spin();

  private:
    const float field_of_view = 0.0349066f;
    //const std::string frame_id = "base_range_";
    sensor_msgs::Range range_msg;

    bool processAck(uint8_t* ack_buffer, const uint8_t* cmd);
    float processRawRangeValue(uint16_t raw_range);
    void processSingleRangeFrame(uint8_t * frame_buffer, int seq);
    void processMultiRangeFrame(uint8_t * frame_buffer, int seq);

    void reconfigure_pixel_mode(
      const teraranger::EvoMiniConfig &config);
    void reconfigure_range_mode(
      const teraranger::EvoMiniConfig &config);

};
}
