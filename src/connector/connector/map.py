import rclpy
import requests
import json
from rclpy import parameter
from rclpy.node import Node


from std_msgs.msg import String


class MapConnector(Node):

    def __init__(self):
        super().__init__('map_connector')
        self.publisher = self.create_publisher(String, '/init_map', 10)
        msg = String()
        api_url = "https://reqbin.com/echo/get/json/page/2"
     
        response = requests.get(api_url)
        msg.data = response.text()
        self.publisher.publish(msg)

        self.subscription = self.create_subscription(
            String,
            '/save_map',
            self.map_callback,
            10)
        self.subscription 

    def map_callback(self, msg):
        api_url = "https://reqbin.com/echo/get/json/page/2"
        parameters={msg}
        response = requests.get(api_url,params=parameters)
        self.get_logger().info('I heard: "%s"' % response.text)


def main(args=None):
    rclpy.init(args=args)

    mapConnector = MapConnector()

    rclpy.spin(mapConnector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mapConnector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()