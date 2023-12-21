#ifndef ULTRASONIC_HC_SR04__ULTRASONIC_HC_SR04_HPP_
#define ULTRASONIC_HC_SR04__ULTRASONIC_HC_SR04_HPP_

#include <inttypes.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>
// C library headers
#include <pigpiod_if2.h>
#include <stdio.h>
#include <string.h>

#include <sensor_msgs/msg/range.hpp>

namespace ultrasonic_sensor
{

#define ECHO_PIN_TIMEOUT_IN_US 1000
#define DEGREE_TO_RAD          M_PI / 180.0
#define MIN_RANGE_IN_M         0.02
#define MAX_RANGE_IN_M         4.0
#define FIELD_OF_VIEW_IN_DEG   15.0
#define SPEED_OF_SOUND_IN_M_S  343.0
#define TRIGGER_PULSE_IN_US    10
#define US_TO_S                0.000001

  class UltrasonicHCSR04 : public rclcpp::Node
  {
   public:
    UltrasonicHCSR04();

   private:
    int _pi;   // pigpio daemon
    std::string _frame_id;
    uint8_t _trigger_pin;
    uint8_t _echo_pin;
    std::string _distance_topic;
    std::string _distance_frame;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr _distance_publisher;
    rclcpp::TimerBase::SharedPtr _timer;

    void connect_to_pigpio_daemon();
    void init_sensor();
    void declare_parameters();
    void setup_publishers();
    void create_wall_timer_for_publisher();
    double get_distance();
    void trigger_sensor();
    bool wait_for_echo_pin_high();
    long get_travel_time_in_us();
    long micros();
    void publish_distance();
  };
}   // namespace ultrasonic_sensor
#endif   // ULTRASONIC_HC_SR04__ULTRASONIC_HC_SR04_HPP_
