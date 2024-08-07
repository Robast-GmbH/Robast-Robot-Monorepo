import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Time

import Jetson.GPIO as GPIO

SENSE_DISINFECTION_SWITCH_PIN = 7

# the switch of the disinfection module closes for 300ms when triggered
BOUNCE_TIME_IN_MS = 300


class DisinfectionPublisher(Node):
    def __init__(self):
        super().__init__("disinfection_publisher")
        self.__declare_parameter()
        self.__get_parameter()
        self.__publisher = self.create_publisher(Time, "disinfection_triggered", 10)
        self.__setup_gpio()
        self.__sum_of_disinfection_readings = 0
        self.__read_in_disinfection_switch_counter = 0

    def __declare_parameter(self):
        self.declare_parameter("read_in_disinfection_switch_counter_limit", 10)
        self.declare_parameter("disinfection_switch_threshold", 0.2)
        self.declare_parameter("is_pulldown", True)
        self.declare_parameter("disinfection_timer_period_in_sec", 0.02)
        # Tests have shown that the first readings after an event is not reliable (probably because of bouncing)
        self.declare_parameter("num_of_first_readings_to_ignore", 1) 

    def __get_parameter(self):
        self.__read_in_disinfection_switch_counter_limit = (
            self.get_parameter("read_in_disinfection_switch_counter_limit")
            .get_parameter_value()
            .integer_value
        )
        self.__disinfection_switch_threshold = (
            self.get_parameter("disinfection_switch_threshold")
            .get_parameter_value()
            .double_value
        )
        self.__timer_period_in_sec = (
            self.get_parameter("disinfection_timer_period_in_sec")
            .get_parameter_value()
            .double_value
        )
        self.__is_pulldown = (
            self.get_parameter("is_pulldown").get_parameter_value().bool_value
        )
        self.__num_of_first_readings_to_ignore = (
            self.get_parameter("num_of_first_readings_to_ignore")
            .get_parameter_value()
            .integer_value
        )

    def destroy_node(self) -> None:
        super().destroy_node()
        GPIO.cleanup()

    def __publish_disinfection_triggered_with_stamp(self):
        msg = self.get_clock().now().to_msg()
        self.get_logger().info(
            f"Disinfection triggered at {msg.sec}.{msg.nanosec} seconds "
            f"because of a running average of "
            f"{self.__average_disinfection_switch_reading} and a threshold of "
            f"{self.__disinfection_switch_threshold}"
        )
        self.__publisher.publish(msg)

    def __trigger_disinfection_evaluation(self, channel: int):
        self.__read_in_disinfection_switch_counter = 0
        self.__sum_of_disinfection_readings = 0
        self.__average_disinfection_switch_reading = 0
        self.get_logger().info(
            f"Disinfection switch triggered on channel {channel}. "
            f"Evaluate disinfection switch for "
            f"{self.__read_in_disinfection_switch_counter_limit} times."
        )
        self.__timer = self.create_timer(
            self.__timer_period_in_sec, self.__evaluate_disinfection_switch
        )

    def __evaluate_disinfection_switch(self):
        self.__read_in_disinfection_switch_pin()
        if (
            self.__read_in_disinfection_switch_counter
            >= self.__read_in_disinfection_switch_counter_limit
        ):
            self.__timer.cancel()
            if self.__is_disinfection_switch_triggered():
                self.__publish_disinfection_triggered_with_stamp()
            else:
                self.get_logger().info(
                    f"False positive detected with running average: {self.__average_disinfection_switch_reading}"
                )

    def __is_disinfection_switch_triggered(self) -> bool:
        self.__average_disinfection_switch_reading = (
            self.__sum_of_disinfection_readings
            / (self.__read_in_disinfection_switch_counter - self.__num_of_first_readings_to_ignore)
        )
        if (
            self.__is_pulldown
            and self.__average_disinfection_switch_reading
            <= self.__disinfection_switch_threshold
        ):
            return True
        elif (
            not self.__is_pulldown
            and self.__average_disinfection_switch_reading
            >= self.__disinfection_switch_threshold
        ):
            return True
        else:
            return False

    def __read_in_disinfection_switch_pin(self):
        self.__read_in_disinfection_switch_counter += 1
        if (
            self.__read_in_disinfection_switch_counter
            <= self.__num_of_first_readings_to_ignore
        ):
            self.get_logger().info(
                f"Ignore reading number {self.__read_in_disinfection_switch_counter} "
                f"because of ignoring first {self.__num_of_first_readings_to_ignore} readings."
            )
            return
        current_disinfection_switch = GPIO.input(SENSE_DISINFECTION_SWITCH_PIN)
        self.__sum_of_disinfection_readings += current_disinfection_switch
        # Print the current state of the switch
        self.get_logger().info(
            f"Current state of the disinfection switch: {current_disinfection_switch} "
            f"for counter {self.__read_in_disinfection_switch_counter}"
        )

    def __setup_gpio(self):
        self.get_logger().info("Setting up GPIO")
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(SENSE_DISINFECTION_SWITCH_PIN, GPIO.IN)
        GPIO.add_event_detect(
            SENSE_DISINFECTION_SWITCH_PIN,
            GPIO.FALLING,
            callback=self.__trigger_disinfection_evaluation,
            bouncetime=BOUNCE_TIME_IN_MS,
        )


def main(args=None):
    rclpy.init(args=args)

    disinfection_publisher = DisinfectionPublisher()

    try:
        rclpy.spin(disinfection_publisher)
        disinfection_publisher.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        disinfection_publisher.destroy_node()


if __name__ == "__main__":
    main()
