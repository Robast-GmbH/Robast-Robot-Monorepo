
import os
import unittest

import ament_index_python
import launch
import launch_testing
import launch_testing.actions
import launch_testing.util
import pytest
import rclpy
import yaml
from launch_ros.actions import Node
from rclpy.qos import (DurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from communication_interfaces.msg import (DrawerAddress, DrawerLeds,
                                          DrawerStatus)


@pytest.mark.launch_test
def generate_test_description():

    # Read input data that is send to dut
    INPUT_DATA_PATH = os.path.join(
    ament_index_python.get_package_prefix('drawer_bridge'),
        'lib/drawer_bridge',
        'node_test_input_data.yaml'
    )

    dut = Node(
        package='drawer_bridge',
        executable='drawer_bridge_test',
        name='drawer_bridge',
    )
    context = {'dut': dut  }

    return (launch.LaunchDescription([   
        dut,
        launch_testing.actions.ReadyToTest()]
        ) , context
    )
    
class TestProcessOutput(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init()


    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

     
    def setUp(self):
        # Create a ROS node for tests
        self.node = rclpy.create_node('drawer_bridge_tester')
        self.received_drawer_status_topic = False
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=2
        )


    def tearDown(self):
        self.node.destroy_node()


    def publish_data(self):
        # Read input data that is send to dut
        INPUT_DATA_PATH = os.path.join(
        ament_index_python.get_package_prefix('drawer_bridge'),
            'lib/drawer_bridge',
            'node_test_input_data.yaml'
        )

        with open(INPUT_DATA_PATH) as f:
            data = yaml.safe_load(f)

        drawer_leds_msg = DrawerLeds()
        drawer_leds_msg.red = data['drawer_leds']['red']
        drawer_leds_msg.blue = data['drawer_leds']['blue']
        drawer_leds_msg.green = data['drawer_leds']['green']
        drawer_leds_msg.brightness = data['drawer_leds']['brightness']
        drawer_leds_msg.mode = data['drawer_leds']['mode']
        self.drawer_leds_publisher_.publish(drawer_leds_msg)
        self.node.get_logger().info('Publishing to drawer_leds topic with led mode: "%i"' % drawer_leds_msg.mode)

        drawer_address_msg = DrawerAddress()
        drawer_address_msg.drawer_controller_id = data['open_drawer']['drawer_controller_id']
        drawer_address_msg.drawer_id = data['open_drawer']['drawer_id']
        self.open_drawer_publisher_.publish(drawer_address_msg)
        self.node.get_logger().info('Publishing to open_drawer topic with drawer_controller_id: "%s"' % drawer_address_msg.drawer_controller_id)


    def drawer_status_subscriber_callback(self, drawer_is_open_msg):
        self.node.get_logger().info('Received msg on drawer_is_open topic. Drawer_controller_id: "%s"' % drawer_is_open_msg.drawer_address.drawer_controller_id)
        self.received_data_drawer_controller_id = drawer_is_open_msg.drawer_address.drawer_controller_id
        self.received_data_drawer_id = drawer_is_open_msg.drawer_address.drawer_id
        self.received_data_drawer_is_open = drawer_is_open_msg.drawer_is_open
        self.received_drawer_status_topic = True

    
    def publish_data_to_dut(self):
        self.open_drawer_publisher_ = self.node.create_publisher(DrawerAddress, 'open_drawer', qos_profile = self.qos_profile)
        self.drawer_leds_publisher_ = self.node.create_publisher(DrawerLeds, 'drawer_leds', qos_profile = self.qos_profile)
        self.publish_data()

    
    def get_expected_result(self):
        # Read data of expected result
        EXPECTED_DATA_PATH = os.path.join(
        ament_index_python.get_package_prefix('drawer_bridge'),
            'lib/drawer_bridge',
            'node_test_expected_data.yaml'
        )

        with open(EXPECTED_DATA_PATH) as f:
            data = yaml.safe_load(f)
        self.expected_data_drawer_controller_id = data['drawer_status']['drawer_address']['drawer_controller_id']
        self.expected_data_drawer_id = data['drawer_status']['drawer_address']['drawer_id']
        self.expected_data_drawer_is_open = data['drawer_status']['drawer_is_open']

    
    def receive_data_from_dut(self):
        self.received_data = []
        self.drawer_status_subscriber = self.node.create_subscription(
            DrawerStatus,
            'drawer_is_open',
            self.drawer_status_subscriber_callback,
            qos_profile=self.qos_profile
        )


    def test_dut_output(self):
        self.publish_data_to_dut()

        self.get_expected_result()

        self.receive_data_from_dut()

        try:
            while not self.received_drawer_status_topic:
                rclpy.spin_once(self.node, timeout_sec=0.1)

            # test actual output for expected output
            # self.assertEqual(str(test_data), expected_data)
            self.assertEqual(self.received_data_drawer_controller_id, self.expected_data_drawer_controller_id)
            self.assertEqual(self.received_data_drawer_id, self.expected_data_drawer_id)
            self.assertEqual(self.received_data_drawer_is_open, self.expected_data_drawer_is_open)

        finally:
            self.node.destroy_publisher(self.open_drawer_publisher_)
            self.node.destroy_publisher(self.drawer_leds_publisher_)
            self.node.destroy_subscription(self.drawer_status_subscriber)