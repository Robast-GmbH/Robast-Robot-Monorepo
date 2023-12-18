#ifndef ULTRASONIC_HC_SR04__ULTRASONIC_HC_SR04_HPP_
#define ULTRASONIC_HC_SR04__ULTRASONIC_HC_SR04_HPP_

#include <inttypes.h>

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
// C library headers
#include <stdio.h>
#include <string.h>

#include <condition_variable>
#include <iostream>
#include <map>
#include <mutex>
#include <queue>
#include <thread>

using namespace std::chrono_literals;

namespace ultrasonic_sensor
{

  class UltrasonicHCSR04 : public rclcpp::Node
  {
   public:
    UltrasonicHCSR04();

   private:
    void setup_publishers();
  };
}   // namespace ultrasonic_sensor
#endif   // ULTRASONIC_HC_SR04__ULTRASONIC_HC_SR04_HPP_
