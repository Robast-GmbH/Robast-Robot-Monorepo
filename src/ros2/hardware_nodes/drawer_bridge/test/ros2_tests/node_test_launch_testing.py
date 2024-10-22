
import os
import unittest
import sys
import threading

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
from communication_interfaces.srv import ModuleConfig, ElectricalDrawerMotorControl

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
        self.__node = rclpy.create_node('drawer_bridge_tester')
        self.__to_can_node = rclpy.create_node('can_receiver')
        self.__received_drawer_feedback_topic = False
        self.__received_drawer_error_feedback_topic = False
        self.__received_data_from_can = []
        self.__qos_profile_open_drawer = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.__qos_profile_led_cmd = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=2
        )
        self.__qos_error_msgs = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        self.__qos_can_msg= QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.setup_subscribers()
        self.setup_publishers()
        self.setup_service_clients()


    def setup_subscribers(self):
        self.__to_can_bus_subscriber = self.__to_can_node.create_subscription(Frame, 'to_can_bus', self.to_can_bus_callback, qos_profile=self.__qos_can_msg)

    def setup_publishers(self):
        self.__open_drawer_publisher = self.__node.create_publisher(DrawerAddress, 'open_drawer', qos_profile = self.__qos_profile_open_drawer)
        self.__led_cmd_publisher = self.__node.create_publisher(LedCmd, 'led_cmd', qos_profile = self.__qos_profile_led_cmd)
        self.__can_in_publisher = self.__node.create_publisher(Frame,'from_can_bus', qos_profile = self.__qos_can_msg)

    def setup_service_clients(self):
        self.__module_config_service_client = self.__node.create_client(ModuleConfig, 'module_config')
        self.__electrical_drawer_motor_control_service_client = self.__node.create_client(ElectricalDrawerMotorControl, 'motor_control')

    def tearDown(self):
        self.__node.destroy_node()


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

        self.__led_cmd_publisher.publish(led_cmd_msg)
        self.__node.get_logger().info('Publishing to led_cmd topic for module_id: "%i"' % led_cmd_msg.drawer_address.module_id)

        drawer_address_msg = DrawerAddress()
        drawer_address_msg.module_id = data['open_drawer']['module_id']
        drawer_address_msg.drawer_id = data['open_drawer']['drawer_id']
        self.__open_drawer_publisher.publish(drawer_address_msg)
        self.__node.get_logger().info('Publishing to open_drawer topic with module_id: "%s"' % drawer_address_msg.module_id)


    def call_service_clients(self):
        # Read input data that is send to dut
        INPUT_DATA_PATH = os.path.join(
        ament_index_python.get_package_prefix('drawer_bridge'),
            'lib/drawer_bridge',
            'node_test_input_data.yaml'
        )

        with open(INPUT_DATA_PATH) as f:
            data = yaml.safe_load(f)

        while not self.__module_config_service_client.wait_for_service(timeout_sec=1.0):
            self.__node.get_logger().info('service not available, waiting again...')
        self.__module_config_request = ModuleConfig.Request()
        self.__module_config_request.module_address.module_id = data['module_config']['module_id']
        self.__module_config_request.module_address.drawer_id = data['module_config']['drawer_id']
        self.__module_config_request.config_id = data['module_config']['config_id']
        self.__module_config_request.config_value = data['module_config']['config_value']
        future = self.__module_config_service_client.call_async(self.__module_config_request)
        rclpy.spin_until_future_complete(self.__node, future)
        self.__module_config_service_response = future.result()

        while not self.__electrical_drawer_motor_control_service_client.wait_for_service(timeout_sec=1.0):
            self.__node.get_logger().info('service not available, waiting again...')
        self.__electrical_drawer_motor_control_request = ElectricalDrawerMotorControl.Request()
        self.__electrical_drawer_motor_control_request.module_address.module_id = data['electrical_drawer_motor_control']['module_id']
        self.__electrical_drawer_motor_control_request.motor_id = data['electrical_drawer_motor_control']['motor_id']
        self.__electrical_drawer_motor_control_request.enable_motor = data['electrical_drawer_motor_control']['enable_motor']
        future = self.__electrical_drawer_motor_control_service_client.call_async(self.__electrical_drawer_motor_control_request)
        rclpy.spin_until_future_complete(self.__node, future)
        self.__electrical_drawer_motor_control_service_response = future.result()


    def publish_drawer_feedback_can_msg(self):
          # Read input data that is send to dut
        INPUT_DATA_PATH = os.path.join(
        ament_index_python.get_package_prefix('drawer_bridge'),
            'lib/drawer_bridge',
            'node_test_input_data.yaml'
        )

        with open(INPUT_DATA_PATH) as f:
            data = yaml.safe_load(f)

        drawer_feedback_can_msg = Frame()
        drawer_feedback_can_msg.id = can_db_defines.CAN_ID_DRAWER_FEEDBACK
        drawer_feedback_can_msg.dlc = can_db_defines.DLC_DRAWER_FEEDBACK
        drawer_feedback_can_msg.data = data['drawer_feedback_can_frame']['data']
        self.__can_in_publisher.publish(drawer_feedback_can_msg)
        self.__node.get_logger().info('Publishing drawer_feedback_can_msg to from_can_bus topic with can_msg_id: "%s"' % drawer_feedback_can_msg.id)


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
        self.__can_in_publisher.publish(error_feedback_can_msg)
        self.__node.get_logger().info('Publishing to from_can_bus topic with can_msg_id: "%s"' % error_feedback_can_msg.id)


    def drawer_feedback_subscriber_callback(self, drawer_is_open_msg):
        self.__node.get_logger().info('Received msg on drawer_is_open topic. module_id: "%s"' % drawer_is_open_msg.drawer_address.module_id)
        self.__received_data_drawer_is_open_module_id = drawer_is_open_msg.drawer_address.module_id
        self.__received_data_drawer_is_open_drawer_id = drawer_is_open_msg.drawer_address.drawer_id
        self.__received_data_drawer_is_open_drawer_is_open = drawer_is_open_msg.drawer_is_open
        self.__received_drawer_feedback_topic = True

    
    def drawer_error_subscriber_callback(self, error_feedback_msg):
        self.__node.get_logger().info('Received msg on robast_error topic. error_code: "%s"' % error_feedback_msg.error_code)
        self.__received_data_error_feedback_error_code = error_feedback_msg.error_code
        self.__received_data_error_feedback_error_data = error_feedback_msg.error_data
        self.__received_drawer_error_feedback_topic = True

    
    def publish_data_to_dut(self):
        self.publish_data()
        self.call_service_clients()

    def to_can_bus_callback(self, msg):
        self.__node.get_logger().info('Received msg on to_can_bus topic. can_id: "%s"' % msg.id)
        self.__received_data_from_can.append(msg)

    
    def get_expected_result(self):
        # Read data of expected result
        EXPECTED_DATA_PATH = os.path.join(
        ament_index_python.get_package_prefix('drawer_bridge'),
            'lib/drawer_bridge',
            'node_test_expected_data.yaml'
        )

        with open(EXPECTED_DATA_PATH) as f:
            data = yaml.safe_load(f)
        self.__expected_data_module_id = data['drawer_feedback']['drawer_address']['module_id']
        self.__expected_data_drawer_id = data['drawer_feedback']['drawer_address']['drawer_id']
        self.__expected_data_drawer_is_open = data['drawer_feedback']['drawer_is_open']

        self.__expected_data_error_feedback_error_code = data['error_feedback']['error_code']
        self.__expected_data_error_feedback_error_data = data['error_feedback']['error_data']

    
    def receive_data_from_dut(self):
        self.__drawer_feedback_subscriber = self.__node.create_subscription(
            DrawerStatus,
            'drawer_is_open',
            self.drawer_feedback_subscriber_callback,
            qos_profile=self.__qos_profile_open_drawer
        )
        self.__robast_error_subscriber = self.__node.create_subscription(
            ErrorBaseMsg,
            'robast_error',
            self.drawer_error_subscriber_callback,
            qos_profile=self.__qos_error_msgs
        )

    
    def check_to_can_bus_data(self):
         # Loop through the received data on the can bus topic and check if the expected data is in the received data
        for msg in self.__received_data_from_can:
            expected_data_bytes = None

            if msg.id == can_db_defines.CAN_ID_MODULE_CONFIG:
                self.assertEqual(msg.dlc, can_db_defines.DLC_MODULE_CONFIG)
                expected_data_uint64 = (
                    (self.__module_config_request.module_address.module_id << (64 - can_db_defines.CAN_SIGNAL_BIT_LENGTH_MODULE_CONFIG_MODULE_ID - can_db_defines.CAN_SIGNAL_BIT_START_MODULE_CONFIG_MODULE_ID)) |
                    (self.__module_config_request.config_id << (64 - can_db_defines.CAN_SIGNAL_BIT_LENGTH_MODULE_CONFIG_CONFIG_ID - can_db_defines.CAN_SIGNAL_BIT_START_MODULE_CONFIG_CONFIG_ID)) |
                    (self.__module_config_request.config_value << (64 - can_db_defines.CAN_SIGNAL_BIT_LENGTH_MODULE_CONFIG_CONFIG_VALUE - can_db_defines.CAN_SIGNAL_BIT_START_MODULE_CONFIG_CONFIG_VALUE))
                )
                expected_data_bytes = expected_data_uint64.to_bytes(8, byteorder='big')

            if msg.id == can_db_defines.CAN_ID_ELECTRICAL_DRAWER_MOTOR_CONTROL:
                self.assertEqual(msg.dlc, can_db_defines.DLC_ELECTRICAL_DRAWER_MOTOR_CONTROL)
                expected_data_uint64 = (
                    (self.__electrical_drawer_motor_control_request.module_address.module_id << (64 - can_db_defines.CAN_SIGNAL_BIT_LENGTH_ELECTRICAL_DRAWER_MOTOR_CONTROL_MODULE_ID - can_db_defines.CAN_SIGNAL_BIT_START_ELECTRICAL_DRAWER_MOTOR_CONTROL_MODULE_ID)) |
                    (self.__electrical_drawer_motor_control_request.motor_id << (64 - can_db_defines.CAN_SIGNAL_BIT_LENGTH_ELECTRICAL_DRAWER_MOTOR_CONTROL_MOTOR_ID - can_db_defines.CAN_SIGNAL_BIT_START_ELECTRICAL_DRAWER_MOTOR_CONTROL_MOTOR_ID)) |
                    (self.__electrical_drawer_motor_control_request.enable_motor << (64 - can_db_defines.CAN_SIGNAL_BIT_LENGTH_ELECTRICAL_DRAWER_MOTOR_CONTROL_ENABLE_MOTOR - can_db_defines.CAN_SIGNAL_BIT_START_ELECTRICAL_DRAWER_MOTOR_CONTROL_ENABLE_MOTOR))
                )
                expected_data_bytes = expected_data_uint64.to_bytes(8, byteorder='big')

            if expected_data_bytes is not None:
                self.__node.get_logger().info('Checking data bytes for can_id: "%s"' % msg.id)
                # Compare each data byte
                for i, byte in enumerate(msg.data):
                    self.assertEqual(byte, expected_data_bytes[i])

    def test_dut_output(self):
        # Create an executor and add to_can_node to spin it in separate thread to receive data from the can bus and proceed with the test
        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.__to_can_node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

        self.publish_data_to_dut()

        self.get_expected_result()

        self.receive_data_from_dut()

        self.publish_drawer_feedback_can_msg()

        self.publish_drawer_error_feedback_can_msg()

        try:
            self.__node.get_logger().info('Starting to compare received data with expected data!')
            while not self.__received_drawer_feedback_topic:
                rclpy.spin_once(self.__node, timeout_sec=0.1)

            # test actual output for expected output
            self.assertEqual(self.__received_data_drawer_is_open_module_id, self.__expected_data_module_id)
            self.assertEqual(self.__received_data_drawer_is_open_drawer_id, self.__expected_data_drawer_id)
            self.assertEqual(self.__received_data_drawer_is_open_drawer_is_open, self.__expected_data_drawer_is_open)
            self.__node.get_logger().info('Finished checking received data from drawer_is_open!')

            while not self.__received_drawer_error_feedback_topic:
                rclpy.spin_once(self.__node, timeout_sec=0.1)

            self.assertEqual(self.__received_data_error_feedback_error_code, self.__expected_data_error_feedback_error_code)
            self.assertEqual(self.__received_data_error_feedback_error_data, self.__expected_data_error_feedback_error_data)
            self.__node.get_logger().info('Finished checking received data from robast_error!')

            # Check if service requests were successful
            self.assertEqual(self.__module_config_service_response.success, True)
            self.assertEqual(self.__electrical_drawer_motor_control_service_response.success, True)

            self.check_to_can_bus_data()
           
        finally:
            self.__node.destroy_publisher(self.__can_in_publisher)
            self.__node.destroy_publisher(self.__open_drawer_publisher)
            self.__node.destroy_publisher(self.__led_cmd_publisher)
            self.__node.destroy_subscription(self.__drawer_feedback_subscriber)
            self.__node.destroy_subscription(self.__robast_error_subscriber)
            self.__node.destroy_client(self.__module_config_service_client)
            self.__node.destroy_client(self.__electrical_drawer_motor_control_service_client)
            self.__to_can_node.destroy_subscription(self.__to_can_bus_subscriber)