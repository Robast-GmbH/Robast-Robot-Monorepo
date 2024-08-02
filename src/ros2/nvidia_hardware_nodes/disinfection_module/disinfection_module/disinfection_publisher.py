import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Time

import Jetson.GPIO as GPIO

SENSE_DISINFECTION_SWITCH_PIN = 7
BOUNCE_TIME_IN_MS = 300 


class DisinfectionPublisher(Node):
    def __init__(self):
        super().__init__("disinfection_publisher")
        self.__publisher = self.create_publisher(Time, "disinfection_triggered", 10)
        self.__setup_gpio()

    def destroy_node(self) -> None:
        super().destroy_node()
        GPIO.cleanup()

    def __publish_disinfection_triggered(self, channel: int):
        msg = self.get_clock().now().to_msg()
        self.get_logger().info(
            f"Disinfection triggered at {msg.sec}.{msg.nanosec} seconds."
        )
        self.__publisher.publish(msg)

    def __setup_gpio(self):
        self.get_logger().info("Setting up GPIO")
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(SENSE_DISINFECTION_SWITCH_PIN, GPIO.IN)
        GPIO.add_event_detect(
            SENSE_DISINFECTION_SWITCH_PIN,
            GPIO.FALLING,
            callback=self.__publish_disinfection_triggered,
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
