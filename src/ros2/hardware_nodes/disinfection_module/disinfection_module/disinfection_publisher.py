import rclpy
import time
from rclpy.node import Node

from builtin_interfaces.msg import Time

import Jetson.GPIO as GPIO

DISINFECTION_PIN = 3

class DisinfectionPublisher(Node):

    def __init__(self):
        super().__init__('disinfection_publisher')
        self._publisher = self.create_publisher(Time, 'disinfection_triggered', 10)
        self.setup_gpio()

    def pub_disinfection_triggered(self):
        msg = Time()
        # fill time message with current timestamp
        msg.sec = time.time()
        msg.nanosec = (time.time() - msg.sec) * 1e9
        self.publisher_.publish(msg)

        self.get_logger().info('Publishing disinfection triggered at: %d.%d' % (msg.sec, msg.nanosec))

    def setup_gpio(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(DISINFECTION_PIN, GPIO.IN)
        GPIO.add_event_detect(DISINFECTION_PIN, GPIO.FALLING, callback=self.pub_disinfection_triggered, bouncetime=10, polltime=0.2)


def main(args=None):
    rclpy.init(args=args)

    disinfection_publisher = DisinfectionPublisher()

    rclpy.spin(disinfection_publisher)

    GPIO.cleanup()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    disinfection_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()