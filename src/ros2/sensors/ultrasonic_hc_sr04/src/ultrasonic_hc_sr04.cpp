#include "ultrasonic_hc_sr04/ultrasonic_hc_sr04.hpp"

namespace ultrasonic_sensor
{
  UltrasonicHCSR04::UltrasonicHCSR04() : Node("drawer_bridge")
  {
    declare_parameters();
    connect_to_pigpio_daemon();
    init_sensor();
    setup_publishers();
    create_wall_timer_for_publisher();
  }

  void UltrasonicHCSR04::declare_parameters()
  {
    // specifies the host or IP address of the Pi running the pigpio daemon.
    this->declare_parameter("pigpio_host", "localhost");
    this->declare_parameter("pigpio_port", "8888");

    this->declare_parameter("trigger_pin", 0);
    this->declare_parameter("echo_pin", 0);
    this->declare_parameter("topic", "distance");
    this->declare_parameter("ultrasonic_sensor_frame", "ultrasonic_sensor_front_middle_frame");
    this->declare_parameter("frequency", 5);
  }

  void UltrasonicHCSR04::connect_to_pigpio_daemon()
  {
    _pi = pigpio_start(this->get_parameter("pigpio_host").as_string().c_str(),
                       this->get_parameter("pigpio_port").as_string().c_str());
    if (_pi < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect to pigpio daemon");
      exit(1);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Successfully connected to pigpio daemon");
    }
  }

  void UltrasonicHCSR04::init_sensor()
  {
    _frame_id = this->get_parameter("ultrasonic_sensor_frame").as_string();
    _trigger_pin = this->get_parameter("trigger_pin").as_int();
    _echo_pin = this->get_parameter("echo_pin").as_int();
    set_mode(_pi, _trigger_pin, PI_OUTPUT);
    set_mode(_pi, _echo_pin, PI_INPUT);
    gpio_write(_pi, _trigger_pin, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  void UltrasonicHCSR04::setup_publishers()
  {
    _distance_publisher = this->create_publisher<sensor_msgs::msg::Range>(this->get_parameter("topic").as_string(), 10);

    RCLCPP_INFO(this->get_logger(),
                "Successfully setup publisher for topic '/%s'",
                this->get_parameter("topic").as_string().c_str());
  }

  void UltrasonicHCSR04::create_wall_timer_for_publisher()
  {
    auto timer_callback = std::bind(&UltrasonicHCSR04::publish_distance, this);
    _timer = this->create_wall_timer(std::chrono::milliseconds(1000 / this->get_parameter("frequency").as_int()),
                                     timer_callback);
  }

  void UltrasonicHCSR04::publish_distance()
  {
    sensor_msgs::msg::Range distance_msg;

    distance_msg.range = get_distance();

    distance_msg.header.frame_id = _frame_id;
    distance_msg.header.stamp = this->now();

    distance_msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;

    if (distance_msg.range == 0.0)
    {
      return;
    }
    _distance_publisher->publish(distance_msg);
  }

  void UltrasonicHCSR04::trigger_sensor()
  {
    // the measurement process is started by sending a 10 microsecond pulse to the trigger input
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    gpio_write(_pi, _trigger_pin, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    gpio_write(_pi, _trigger_pin, 0);
  }

  bool UltrasonicHCSR04::wait_for_echo_pin_high()
  {
    long trigger_timestamp_in_us = micros();

    bool is_echo_pin_low = (gpio_read(_pi, _echo_pin) == 0);
    bool is_timeout_reached = (micros() - trigger_timestamp_in_us) > ECHO_PIN_TIMEOUT_IN_US;
    while (is_echo_pin_low && !is_timeout_reached)
    {
    }

    if (is_timeout_reached)
    {
      RCLCPP_WARN(this->get_logger(),
                  "Timeout reached for ultrasonic sensor with frame_id %s. Echo pin was not set to high within %d ms!",
                  _frame_id.c_str(),
                  ECHO_PIN_TIMEOUT_IN_US / 1000);
      return false;
    }

    return true;
  }

  double UltrasonicHCSR04::get_distance()
  {
    // the measurement process is started by sending a 10 microsecond pulse to the trigger input
    // This triggers the ultrasonic burst which travels through the air
    trigger_sensor();

    // Once the burst is sent the echo pin goes high, and stays high until the echo is received back.
    bool set_echo_pin_set_high_successfully = wait_for_echo_pin_high();
    if (!set_echo_pin_set_high_successfully)
    {
      return 0.0;
    }

    // Therefore we can measure the time the echo pin is high to calculate the distance
    long travel_time_in_us = get_travel_time_in_us();

    double distance_in_meters = ((travel_time_in_us / 1000000.0) * 340.29) / 2;

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
    while (gpio_read(_pi, _echo_pin) == 1)
    {
    }
    long end_time_us = micros();
    return (end_time_us - start_time_us);
  }

}   // namespace ultrasonic_sensor
