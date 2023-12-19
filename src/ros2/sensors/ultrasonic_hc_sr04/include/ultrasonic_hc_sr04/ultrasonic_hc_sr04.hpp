#ifndef ULTRASONIC_HC_SR04__ULTRASONIC_HC_SR04_HPP_
#define ULTRASONIC_HC_SR04__ULTRASONIC_HC_SR04_HPP_

#include <inttypes.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>
// C library headers
#include <pigpio.h>
#include <stdio.h>
#include <string.h>

#include <std_msgs/msg/float32.hpp>

namespace ultrasonic_sensor
{

  class UltrasonicHCSR04 : public rclcpp::Node
  {
   public:
    UltrasonicHCSR04();

   private:
    uint8_t _trigger_pin;
    uint8_t _echo_pin;
    std::string _distance_topic;
    std::string _distance_frame;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _distance_publisher;

    void setup_publishers();
    void declare_parameters();
    void init_sensor();
    double get_distance(uint32_t timeout);
    long get_travel_time_in_us();
    long micros();
  };
}   // namespace ultrasonic_sensor
#endif   // ULTRASONIC_HC_SR04__ULTRASONIC_HC_SR04_HPP_
