
import os
import unittest
import sys


current_script_dir = os.path.dirname(__file__)
workspace_dir = os.path.abspath(os.path.join(current_script_dir, '..', '..', '..', '..', '..'))
sys.path.append(os.path.join(workspace_dir, 'build', 'can')) # Add the build directory to the Python path
import can_db_defines_bindings as can_db_defines

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

from communication_interfaces.msg import (DrawerAddress, Led, LedCmd,
                                          DrawerStatus, ErrorBaseMsg)

from can_msgs.msg import Frame


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
    context = {'dut': dut}

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
        self.received_drawer_error_feedback_topic = False
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

        led_msg = Led()
        led_msg.red = data['led']['red']
        led_msg.blue = data['led']['blue']
        led_msg.green = data['led']['green']
        led_msg.brightness = data['led']['brightness']

        led_cmd_msg = LedCmd()
        led_cmd_msg.drawer_address.module_id = data['led_cmd']['drawer_address']['module_id']
        led_cmd_msg.drawer_address.drawer_id = data['led_cmd']['drawer_address']['drawer_id']
        led_cmd_msg.leds = [led_msg, led_msg]
        led_cmd_msg.start_index = data['led_cmd']['start_index']

        self.led_cmd_publisher_.publish(led_cmd_msg)
        self.node.get_logger().info('Publishing to led_cmd topic for module_id: "%i"' % led_cmd_msg.drawer_address.module_id)

        drawer_address_msg = DrawerAddress()
        drawer_address_msg.module_id = data['open_drawer']['module_id']
        drawer_address_msg.drawer_id = data['open_drawer']['drawer_id']
        self.open_drawer_publisher_.publish(drawer_address_msg)
        self.node.get_logger().info('Publishing to open_drawer topic with module_id: "%s"' % drawer_address_msg.module_id)


    def publish_drawer_feedback_can_msg(self):
          # Read input data that is send to dut
        INPUT_DATA_PATH = os.path.join(
        ament_index_python.get_package_prefix('drawer_bridge'),
            'lib/drawer_bridge',
            'node_test_input_data.yaml'
        )

        with open(INPUT_DATA_PATH) as f:
            data = yaml.safe_load(f)

        drawer_is_open_can_msg = Frame()
        drawer_is_open_can_msg.id = can_db_defines.CAN_ID_DRAWER_FEEDBACK
        drawer_is_open_can_msg.dlc = can_db_defines.DLC_DRAWER_FEEDBACK
        drawer_is_open_can_msg.data = data['drawer_feedback_can_frame']['data']
        self.can_in_publisher_.publish(drawer_is_open_can_msg)
        self.node.get_logger().info('Publishing drawer_is_open_can_msg to from_can_bus topic with can_msg_id: "%s"' % drawer_is_open_can_msg.id)


    def publish_drawer_error_feedback_can_msg(self):
          # Read input data that is send to dut
        INPUT_DATA_PATH = os.path.join(
        ament_index_python.get_package_prefix('drawer_bridge'),
            'lib/drawer_bridge',
            'node_test_input_data.yaml'
        )

        with open(INPUT_DATA_PATH) as f:
            data = yaml.safe_load(f)

        error_feedback_can_msg = Frame()
        error_feedback_can_msg.id = can_db_defines.CAN_ID_ERROR_FEEDBACK
        error_feedback_can_msg.dlc = can_db_defines.DLC_ERROR_FEEDBACK
        error_feedback_can_msg.data = data['error_feedback_can_frame']['data']
        self.can_in_publisher_.publish(error_feedback_can_msg)
        self.node.get_logger().info('Publishing to from_can_bus topic with can_msg_id: "%s"' % error_feedback_can_msg.id)


    def drawer_status_subscriber_callback(self, drawer_is_open_msg):
        self.node.get_logger().info('Received msg on drawer_is_open topic. module_id: "%s"' % drawer_is_open_msg.drawer_address.module_id)
        self.received_data_drawer_is_open_module_id = drawer_is_open_msg.drawer_address.module_id
        self.received_data_drawer_is_open_drawer_id = drawer_is_open_msg.drawer_address.drawer_id
        self.received_data_drawer_is_open_drawer_is_open = drawer_is_open_msg.drawer_is_open
        self.received_drawer_status_topic = True

    
    def drawer_error_subscriber_callback(self, error_feedback_msg):
        self.node.get_logger().info('Received msg on robast_error topic. error_code: "%s"' % error_feedback_msg.error_code)
        self.received_data_error_feedback_error_code = error_feedback_msg.error_code
        self.received_data_error_feedback_error_data = error_feedback_msg.error_data
        self.received_drawer_error_feedback_topic = True

    
    def publish_data_to_dut(self):
        self.open_drawer_publisher_ = self.node.create_publisher(DrawerAddress, 'open_drawer', qos_profile = self.qos_profile)
        self.led_cmd_publisher_ = self.node.create_publisher(LedCmd, 'led_cmd', qos_profile = self.qos_profile)
        self.can_in_publisher_ = self.node.create_publisher(Frame,'from_can_bus', qos_profile = self.qos_profile)
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
        self.expected_data_module_id = data['drawer_status']['drawer_address']['module_id']
        self.expected_data_drawer_id = data['drawer_status']['drawer_address']['drawer_id']
        self.expected_data_drawer_is_open = data['drawer_status']['drawer_is_open']

        self.expected_data_error_feedback_error_code = data['error_feedback']['error_code']
        self.expected_data_error_feedback_error_data = data['error_feedback']['error_data']

    
    def receive_data_from_dut(self):
        self.received_data = []
        self.drawer_status_subscriber = self.node.create_subscription(
            DrawerStatus,
            'drawer_is_open',
            self.drawer_status_subscriber_callback,
            qos_profile=self.qos_profile
        )
        self.drawer_status_subscriber = self.node.create_subscription(
            ErrorBaseMsg,
            'robast_error',
            self.drawer_error_subscriber_callback,
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=10
            )
        )

   

    def test_dut_output(self):
        self.publish_data_to_dut()

        self.get_expected_result()

        self.receive_data_from_dut()

        self.publish_drawer_feedback_can_msg()

        self.publish_drawer_error_feedback_can_msg()

        try:
            self.node.get_logger().info('Starting to compare received data with expected data!')
            while not self.received_drawer_status_topic:
                rclpy.spin_once(self.node, timeout_sec=0.1)

            # test actual output for expected output
            # self.assertEqual(str(test_data), expected_data)
            self.assertEqual(self.received_data_drawer_is_open_module_id, self.expected_data_module_id)
            self.assertEqual(self.received_data_drawer_is_open_drawer_id, self.expected_data_drawer_id)
            self.assertEqual(self.received_data_drawer_is_open_drawer_is_open, self.expected_data_drawer_is_open)
            self.node.get_logger().info('Finished checking received data from drawer_is_open!')

            while not self.received_drawer_error_feedback_topic:
                rclpy.spin_once(self.node, timeout_sec=0.1)

            self.assertEqual(self.received_data_error_feedback_error_code, self.expected_data_error_feedback_error_code)
            self.assertEqual(self.received_data_error_feedback_error_data, self.expected_data_error_feedback_error_data)
            self.node.get_logger().info('Finished checking received data from robast_error!')



        finally:
            self.node.destroy_publisher(self.open_drawer_publisher_)
            self.node.destroy_publisher(self.led_cmd_publisher_)
            self.node.destroy_subscription(self.drawer_status_subscriber)