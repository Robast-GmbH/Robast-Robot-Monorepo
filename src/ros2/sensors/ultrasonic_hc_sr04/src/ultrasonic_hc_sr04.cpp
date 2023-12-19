#include "ultrasonic_hc_sr04/ultrasonic_hc_sr04.hpp"

namespace ultrasonic_sensor
{
  UltrasonicHCSR04::UltrasonicHCSR04() : Node("drawer_bridge")
  {
    setup_publishers();
    declare_parameters();
    init_sensor();
  }

  void UltrasonicHCSR04::declare_parameters()
  {
    // Declare parameters
    this->declare_parameter("trigger_pin", 0);
    this->declare_parameter("echo_pin", 0);
    this->declare_parameter("distance_topic", "ultrasonic_distance");
    this->declare_parameter("distance_frame", "ultrasonic_distance_frame");
  }

  void UltrasonicHCSR04::init_sensor()
  {
    _trigger_pin = this->get_parameter("trigger_pin").as_int();
    _echo_pin = this->get_parameter("echo_pin").as_int();
    gpioSetMode(_trigger_pin, PI_OUTPUT);
    gpioSetMode(_echo_pin, PI_INPUT);
    gpioWrite(_trigger_pin, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  double UltrasonicHCSR04::get_distance(uint32_t timeout)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    gpioWrite(_trigger_pin, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    gpioWrite(_trigger_pin, 0);

    long trigger_timestamp = micros();

    while (gpioRead(_echo_pin) == 0 && micros() - trigger_timestamp < timeout)
    {
    }

    long travel_time_us = get_travel_time_in_us();

    double distance_in_meters = 100 * ((travel_time_us / 1000000.0) * 340.29) / 2;

    return distance_in_meters;
  }

  long UltrasonicHCSR04::micros()
  {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::high_resolution_clock::now().time_since_epoch())
        .count();
  }

  long UltrasonicHCSR04::get_travel_time_in_us()
  {
    long start_time_us = micros();
    while (gpioRead(_echo_pin) == 1)
    {
    }
    long end_time_us = micros();
    return (end_time_us - start_time_us);
  }

  void UltrasonicHCSR04::setup_publishers()
  {
    // Add a publisher to publish the results of the ultrasonic sensor
    _distance_publisher =
        this->create_publisher<std_msgs::msg::Float32>(this->get_parameter("distance_topic").as_string(), 10);
  }

}   // namespace ultrasonic_sensor
