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
#define EVO_MINI_MAX_RANGE_SINGLE_LONG 3.3
#define EVO_MINI_MAX_RANGE_TWO_PIXEL_SHORT 2.4
#define EVO_MINI_MAX_RANGE_TWO_PIXEL_LONG 2.4
#define EVO_MINI_MAX_RANGE_TWO_BY_TWO_SHORT 1.35
#define EVO_MINI_MAX_RANGE_TWO_BY_TWO_LONG 1.65

#define EVO_MINI_MIN_RANGE_SINGLE_SHORT 0.03
#define EVO_MINI_MIN_RANGE_SINGLE_LONG 0.03
#define EVO_MINI_MIN_RANGE_TWO_PIXEL_SHORT 0.03
#define EVO_MINI_MIN_RANGE_TWO_PIXEL_LONG 0.03
#define EVO_MINI_MIN_RANGE_TWO_BY_TWO_SHORT 0.03
#define EVO_MINI_MIN_RANGE_TWO_BY_TWO_LONG 0.03

#define EVO_MINI_SINGLE_PIXEL_FOV 0.34906585039
#define EVO_MINI_TWO_PIXEL_FOV 0.4
#define EVO_MINI_TWO_BY_TWO_FOV 0.47123889803

#define EVO_MINI_MAX_PIXEL_COUNT 4

#define SERIAL_SPEED 115200
#define SERIAL_TIMEOUT_MS 1000

#define ACK_HEADER 0x12
#define ACK_VALUE 0x00
#define NACK_VALUE 0xFF
#define ACK_LENGTH 4

#define BUFFER_SIZE 10
#define RANGE_FRAME_HEADER 'T'
#define RANGE_FRAME_LENGTH_SINGLE_PIXEL 4
#define RANGE_FRAME_LENGTH_TWO_PIXEL 6
#define RANGE_FRAME_LENGTH_TWO_BY_TWO 10

namespace teraranger
{
static const char ENABLE_CMD[5] = {(char)0x00, (char)0x52, (char)0x02, (char)0x01, (char)0xDF};

static const char TEXT_MODE[4] = {(char)0x00, (char)0x11, (char)0x01, (char)0x45};
static const char BINARY_MODE[4] = {(char)0x00, (char)0x11, (char)0x02, (char)0x4C};
static const char SINGLE_PIXEL_MODE[4] = {(char)0x00, (char)0x21, (char)0x01, (char)0xBC};
static const char TWO_PIXEL_MODE[4] = {(char)0x00, (char)0x21, (char)0x03, (char)0xB2};
static const char TWO_BY_TWO_PIXEL_MODE[4] = {(char)0x00, (char)0x21, (char)0x02, (char)0xB5};
static const char LONG_RANGE_MODE[4] = {(char)0x00, (char)0x61, (char)0x03, (char)0xE9};
static const char SHORT_RANGE_MODE[4] = {(char)0x00, (char)0x61, (char)0x01, (char)0xE7};

class TerarangerEvoMini
{
  public:
    TerarangerEvoMini();
    virtual ~TerarangerEvoMini();

    void spin();
  private:
    std::string portname_;
    std::string frame_id_;
    std::string ns_;
    ros::NodeHandle nh_;
    ros::Publisher range_publisher_;
    ros::Publisher ranges_publisher_;

    serial::Serial serial_port_;
    boost::function<void(uint8_t)> serial_data_callback_function_;
    void serialDataCallback(uint8_t data);
    void setMode(const char *c, int length);

    void dynParamCallback(const teraranger::EvoMiniConfig &config, uint32_t level);
    dynamic_reconfigure::Server<teraranger::EvoMiniConfig> dyn_param_server_;
    dynamic_reconfigure::Server<teraranger::EvoMiniConfig>::CallbackType dyn_param_server_callback_function_;
    int current_pixel_mode = teraranger::EvoMini_Single_pixel;
    int current_range_mode = teraranger::EvoMini_Long;
    void updateExtrema();

    sensor_msgs::Range range_msg;
    teraranger_array::RangeArray range_array_msg;
    float current_min;
    float current_max;
    float current_fov;

    void checkSubscribers(bool multi);

    bool processAck(uint8_t* ack_buffer, const uint8_t* cmd);
    void initRangeArrayMessage(int nb_ranges);
    float processRawRangeValue(uint16_t raw_range);
    void processSingleRangeFrame(uint8_t * frame_buffer, int seq);
    void processMultiRangeFrame(uint8_t * frame_buffer, int seq, int range_count);

    void reconfigurePixelMode(
      const teraranger::EvoMiniConfig &config);
    void reconfigureRangeMode(
      const teraranger::EvoMiniConfig &config);
};
}
