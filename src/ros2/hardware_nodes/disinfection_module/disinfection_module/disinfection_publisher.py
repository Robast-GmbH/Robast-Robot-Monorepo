import rclpy
import time
from rclpy.node import Node

from builtin_interfaces.msg import Time

import Jetson.GPIO as GPIO

DISINFECTION_PIN = 7
BOUNCE_TIME_IN_MS = 300

class DisinfectionPublisher(Node):

    def __init__(self):
        super().__init__('disinfection_publisher')
        self.__publisher = self.create_publisher(Time, 'disinfection_triggered', 10)
        self.setup_gpio()


    def pub_disinfection_triggered(self, channel):
        msg = Time()
        # fill time message with current timestamp
        current_time = time.time()
        msg.sec = int(current_time)
        msg.nanosec = int((current_time - msg.sec) * 1e9)
        self.get_logger().info('Publishing disinfection triggered at: %d.%d' % (msg.sec, msg.nanosec))
        self.__publisher.publish(msg)


    def setup_gpio(self):
        self.get_logger().info('Setting up GPIO')
        self.get_logger().info('GPIO.JETSON_INFO = %s' % GPIO.JETSON_INFO)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(DISINFECTION_PIN, GPIO.IN)
        GPIO.add_event_detect(DISINFECTION_PIN, GPIO.FALLING, callback=self.pub_disinfection_triggered, bouncetime=BOUNCE_TIME_IN_MS)


def main(args=None):
    rclpy.init(args=args)

    disinfection_publisher = DisinfectionPublisher()

    rclpy.spin(disinfection_publisher)

    GPIO.cleanup()

    disinfection_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()