import os
import unittest
import sys
import threading
import time

current_script_dir = os.path.dirname(__file__)
workspace_dir = os.path.abspath(os.path.join(current_script_dir, "..", "..", "..", "..", ".."))
sys.path.append(os.path.join(workspace_dir, "build", "can"))  # Add the build directory to the Python path
sys.path.append(current_script_dir)
import can_db_defines_bindings as can_db_defines
import can_data_helpers

import ament_index_python
import launch
import launch_testing
import launch_testing.actions
import launch_testing.util
import pytest
import rclpy
import yaml
from launch_ros.actions import Node
from rclpy.action import ActionClient

from rclpy.qos import (
    DurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from communication_interfaces.msg import (
    DrawerAddress,
    Led,
    LedCmd,
    DrawerStatus,
    ErrorBaseMsg,
    Heartbeat,
)
from communication_interfaces.action import ModuleConfig, ElectricalDrawerMotorControl

from can_msgs.msg import Frame


@pytest.mark.launch_test
def generate_test_description():

    # Read input data that is send to dut
    INPUT_DATA_PATH = os.path.join(
        ament_index_python.get_package_prefix("drawer_bridge"),
        "lib/drawer_bridge",
        "node_test_input_data.yaml",
    )

    dut = Node(
        package="drawer_bridge",
        executable="drawer_bridge_test",
        name="drawer_bridge",
    )
    context = {"dut": dut}

    return (
        launch.LaunchDescription([dut, launch_testing.actions.ReadyToTest()]),
        context,
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
        self.__node = rclpy.create_node("drawer_bridge_tester")
        self.__can_node = rclpy.create_node("can_receiver")
        self.__received_drawer_feedback_topic = False
        self.__received_drawer_error_feedback_topic = False
        self.__received_heartbeat_topic = False
        self.__received_data_from_can = []
        self.__qos_profile_open_drawer = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.__qos_profile_led_cmd = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=2,
        )
        self.__qos_error_msgs = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10,
        )
        self.__qos_can_msg = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self.setup_subscribers()
        self.setup_publishers()
        self.setup_action_clients()

    def setup_subscribers(self):
        self.__to_can_bus_subscriber = self.__can_node.create_subscription(
            Frame,
            "to_can_bus",
            self.to_can_bus_callback,
            qos_profile=self.__qos_can_msg,
        )
        self.__drawer_feedback_subscriber = self.__node.create_subscription(
            DrawerStatus,
            "drawer_is_open",
            self.drawer_feedback_subscriber_callback,
            qos_profile=self.__qos_profile_open_drawer,
        )
        self.__robast_error_subscriber = self.__node.create_subscription(
            ErrorBaseMsg,
            "robast_error",
            self.drawer_error_subscriber_callback,
            qos_profile=self.__qos_error_msgs,
        )
        self.__heartbeat_subscriber = self.__node.create_subscription(
            Heartbeat,
            "heartbeat",
            self.heartbeat_callback,
            qos_profile=self.__qos_error_msgs,
        )

    def setup_publishers(self):
        self.__open_drawer_publisher = self.__node.create_publisher(
            DrawerAddress, "open_drawer", qos_profile=self.__qos_profile_open_drawer
        )
        self.__led_cmd_publisher = self.__node.create_publisher(
            LedCmd, "led_cmd", qos_profile=self.__qos_profile_led_cmd
        )
        self.__led_cmd_safety_publisher = self.__node.create_publisher(
            LedCmd, "safety/led_cmd", qos_profile=self.__qos_profile_led_cmd
        )
        self.__can_in_publisher = self.__can_node.create_publisher(
            Frame, "from_can_bus", qos_profile=self.__qos_can_msg
        )

    def setup_action_clients(self):
        self.__module_config_service_client = ActionClient(self.__node, ModuleConfig, "module_config")
        self.__electrical_drawer_motor_control_service_client = ActionClient(
            self.__node, ElectricalDrawerMotorControl, "motor_control"
        )

    def tearDown(self):
        self.__node.destroy_node()

    def publish_ros_data(self):
        # Read input data that is send to dut
        INPUT_DATA_PATH = os.path.join(
            ament_index_python.get_package_prefix("drawer_bridge"),
            "lib/drawer_bridge",
            "node_test_input_data.yaml",
        )

        with open(INPUT_DATA_PATH) as f:
            data = yaml.safe_load(f)

        self.publish_led_cmd_msg_without_ack_requested(data)
        self.__node.get_logger().info("Sleeping for 0.5 second to give time to publish the data...")
        time.sleep(0.5)
        self.publish_led_cmd_msg_with_ack_requested(data)
        self.__node.get_logger().info("Sleeping for 0.5 second to give time to publish the data...")
        time.sleep(0.5)

        self.__open_drawer_msg = DrawerAddress()
        self.__open_drawer_msg.module_id = data["open_drawer"]["module_id"]
        self.__open_drawer_msg.drawer_id = data["open_drawer"]["drawer_id"]
        self.__open_drawer_publisher.publish(self.__open_drawer_msg)
        self.__node.get_logger().info(
            'Publishing to open_drawer topic with module_id: "%s"' % self.__open_drawer_msg.module_id
        )

    def publish_led_cmd_msg_without_ack_requested(self, data):
        led_state_1_msg = Led()
        led_state_1_msg.red = data["led_state_1"]["red"]
        led_state_1_msg.blue = data["led_state_1"]["blue"]
        led_state_1_msg.green = data["led_state_1"]["green"]
        led_state_1_msg.brightness = data["led_state_1"]["brightness"]

        led_state_2_msg = Led()
        led_state_2_msg.red = data["led_state_2"]["red"]
        led_state_2_msg.blue = data["led_state_2"]["blue"]
        led_state_2_msg.green = data["led_state_2"]["green"]
        led_state_2_msg.brightness = data["led_state_2"]["brightness"]

        self.__led_states = [led_state_1_msg, led_state_2_msg, led_state_1_msg, led_state_2_msg]
        self.__num_of_led_state_msgs = [1, 5, 3, 4]
        self.__ack_requested = []

        self.__led_cmd_msg = LedCmd()
        self.__led_cmd_msg.drawer_address.module_id = data["led_cmd"]["drawer_address"]["module_id"]
        self.__led_cmd_msg.drawer_address.drawer_id = data["led_cmd"]["drawer_address"]["drawer_id"]
        self.__led_cmd_msg.start_index = data["led_cmd"]["start_index"]
        self.__led_cmd_msg.fade_time_in_ms = data["led_cmd"]["fade_time_in_ms"]

        for i, num_msgs in enumerate(self.__num_of_led_state_msgs):
            self.__ack_requested.append(False)
            for _ in range(num_msgs):
                self.__led_cmd_msg.leds.append(self.__led_states[i])

        self.__led_cmd_publisher.publish(self.__led_cmd_msg)
        self.__node.get_logger().info(
            'Publishing to led_cmd topic for module_id: "%i"' % self.__led_cmd_msg.drawer_address.module_id
        )

    def publish_led_cmd_msg_with_ack_requested(self, data):
        led_state_msg = Led()
        led_state_msg.red = data["led_state_1"]["red"]
        led_state_msg.blue = data["led_state_1"]["blue"]
        led_state_msg.green = data["led_state_1"]["green"]

        self.__led_cmd_msg_ack_requested = LedCmd()
        self.__led_cmd_msg_ack_requested.drawer_address.module_id = data["led_cmd"]["drawer_address"]["module_id"]
        self.__led_cmd_msg_ack_requested.drawer_address.drawer_id = data["led_cmd"]["drawer_address"]["drawer_id"]
        self.__led_cmd_msg_ack_requested.start_index = data["led_cmd"]["start_index"]
        self.__led_cmd_msg_ack_requested.fade_time_in_ms = data["led_cmd"]["fade_time_in_ms"]
        self.__led_cmd_msg_ack_requested.leds.append(led_state_msg)

        self.__led_states.append(led_state_msg)
        self.__num_of_led_state_msgs.append(1)
        self.__ack_requested.append(True)

        self.__led_cmd_safety_publisher.publish(self.__led_cmd_msg_ack_requested)
        self.__node.get_logger().info(
            'Publishing to led_cmd topic for module_id: "%i" with ack requested'
            % self.__led_cmd_msg_ack_requested.drawer_address.module_id
        )
        # Send message to can_in topic to mimic the response from the dut
        acknowledgment_msg = can_data_helpers.construct_acknowledgment_can_frame(
            self.__led_cmd_msg_ack_requested.drawer_address.module_id, can_db_defines.can_id.LED_STATE
        )
        self.__can_in_publisher.publish(acknowledgment_msg)

    def call_action_clients(self):
        # Read input data that is send to dut
        INPUT_DATA_PATH = os.path.join(
            ament_index_python.get_package_prefix("drawer_bridge"),
            "lib/drawer_bridge",
            "node_test_input_data.yaml",
        )

        with open(INPUT_DATA_PATH) as f:
            data = yaml.safe_load(f)

        while not self.__module_config_service_client.wait_for_server(timeout_sec=1.0):
            self.__node.get_logger().info("Action server not available, waiting again...")
        self.__module_config_goal = ModuleConfig.Goal()
        self.__module_config_goal.module_address.module_id = data["module_config"]["module_id"]
        self.__module_config_goal.module_address.drawer_id = data["module_config"]["drawer_id"]
        self.__module_config_goal.config_id = data["module_config"]["config_id"]
        self.__module_config_goal.config_value = data["module_config"]["config_value"]
        send_goal_future = self.__module_config_service_client.send_goal_async(self.__module_config_goal)
        rclpy.spin_until_future_complete(self.__node, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.__node.get_logger().info("Setting config: Goal rejected!")
            return
        self.__node.get_logger().info("Setting config: Goal accepted!")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.__node, result_future)
        self.__setting_module_config_succeeded = result_future.result().result.success

        while not self.__electrical_drawer_motor_control_service_client.wait_for_server(timeout_sec=1.0):
            self.__node.get_logger().info("Action server not available, waiting again...")
        self.__electrical_drawer_motor_control_goal = ElectricalDrawerMotorControl.Goal()
        self.__electrical_drawer_motor_control_goal.module_address.module_id = data["electrical_drawer_motor_control"][
            "module_id"
        ]
        self.__electrical_drawer_motor_control_goal.motor_id = data["electrical_drawer_motor_control"]["motor_id"]
        self.__electrical_drawer_motor_control_goal.enable_motor = data["electrical_drawer_motor_control"][
            "enable_motor"
        ]
        send_goal_future = self.__electrical_drawer_motor_control_service_client.send_goal_async(
            self.__electrical_drawer_motor_control_goal
        )
        rclpy.spin_until_future_complete(self.__node, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.__node.get_logger().info("Setting motor control: Goal rejected!")
            return
        self.__node.get_logger().info("Setting motor control: Goal accepted!")

        # Send message to can_in topic to mimic the response from the dut
        electrical_drawer_motor_control_feedback = can_data_helpers.construct_e_drawer_motor_control_feedback_can_frame(
            self.__electrical_drawer_motor_control_goal
        )
        self.__can_in_publisher.publish(electrical_drawer_motor_control_feedback)

        # Get the result of the action client
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.__node, result_future)
        self.__setting_electrical_drawer_motor_control_succeeded = result_future.result().result.success

    def publish_data_to_can_bus(self):
        self.publish_drawer_feedback_can_msg()
        self.publish_drawer_error_feedback_can_msg()
        self.publish_heartbeat_can_msg()

    def publish_drawer_feedback_can_msg(self):
        # Read input data that is send to dut
        INPUT_DATA_PATH = os.path.join(
            ament_index_python.get_package_prefix("drawer_bridge"),
            "lib/drawer_bridge",
            "node_test_input_data.yaml",
        )

        with open(INPUT_DATA_PATH) as f:
            data = yaml.safe_load(f)

        drawer_feedback_can_msg = Frame()
        drawer_feedback_can_msg.id = can_db_defines.can_id.DRAWER_FEEDBACK
        drawer_feedback_can_msg.dlc = can_db_defines.can_dlc.DRAWER_FEEDBACK
        drawer_feedback_can_msg.data = data["drawer_feedback_can_frame"]["data"]
        self.__can_in_publisher.publish(drawer_feedback_can_msg)
        self.__node.get_logger().info(
            'Publishing drawer_feedback_can_msg to from_can_bus topic with can_msg_id: "%s"'
            % drawer_feedback_can_msg.id
        )

    def publish_drawer_error_feedback_can_msg(self):
        # Read input data that is send to dut
        INPUT_DATA_PATH = os.path.join(
            ament_index_python.get_package_prefix("drawer_bridge"),
            "lib/drawer_bridge",
            "node_test_input_data.yaml",
        )

        with open(INPUT_DATA_PATH) as f:
            data = yaml.safe_load(f)

        error_feedback_can_msg = Frame()
        error_feedback_can_msg.id = can_db_defines.can_id.ERROR_FEEDBACK
        error_feedback_can_msg.dlc = can_db_defines.can_dlc.ERROR_FEEDBACK
        error_feedback_can_msg.data = data["error_feedback_can_frame"]["data"]
        self.__can_in_publisher.publish(error_feedback_can_msg)
        self.__node.get_logger().info(
            'Publishing to from_can_bus topic with can_msg_id: "%s"' % error_feedback_can_msg.id
        )

    def publish_heartbeat_can_msg(self):
        self.__heartbeat_module_id = 0x010203
        self.__heartbeat_interval_in_ms = 1000
        heartbeat_can_msg = can_data_helpers.construct_heartbeat_can_frame(
            self.__heartbeat_module_id, self.__heartbeat_interval_in_ms
        )
        self.__can_in_publisher.publish(heartbeat_can_msg)
        self.__node.get_logger().info(
            'Publishing Heartbeat to from_can_bus topic with can_msg_id: "%s"' % heartbeat_can_msg.id
        )

    def drawer_feedback_subscriber_callback(self, drawer_is_open_msg):
        self.__node.get_logger().info(
            'Received msg on drawer_is_open topic. module_id: "%s"' % drawer_is_open_msg.drawer_address.module_id
        )
        self.__received_data_drawer_is_open_module_id = drawer_is_open_msg.drawer_address.module_id
        self.__received_data_drawer_is_open_drawer_id = drawer_is_open_msg.drawer_address.drawer_id
        self.__received_data_drawer_is_open_drawer_is_open = drawer_is_open_msg.drawer_is_open
        self.__received_drawer_feedback_topic = True

    def drawer_error_subscriber_callback(self, error_feedback_msg):
        self.__node.get_logger().info(
            'Received msg on robast_error topic. error_code: "%s"' % error_feedback_msg.error_code
        )
        self.__received_data_error_feedback_error_code = error_feedback_msg.error_code
        self.__received_data_error_feedback_error_data = error_feedback_msg.error_data
        self.__received_drawer_error_feedback_topic = True

    def to_can_bus_callback(self, msg):
        self.__node.get_logger().info('Received msg on to_can_bus topic. can_id: "%s"' % msg.id)
        self.__received_data_from_can.append(msg)

    def heartbeat_callback(self, msg):
        self.__node.get_logger().info('Received msg on heartbeat topic. Id: "%s"' % msg.id)
        self.__received_data_heartbeat_id = int(msg.id)
        self.__received_data_interval_in_ms = msg.interval_in_ms
        self.__received_data_stamp = msg.stamp
        self.__received_heartbeat_topic = True

    def publish_data_to_dut(self):
        self.publish_ros_data()
        self.call_action_clients()
        self.publish_data_to_can_bus()

    def get_expected_results(self):
        # Read data of expected result
        EXPECTED_DATA_PATH = os.path.join(
            ament_index_python.get_package_prefix("drawer_bridge"),
            "lib/drawer_bridge",
            "node_test_expected_data.yaml",
        )

        with open(EXPECTED_DATA_PATH) as f:
            data = yaml.safe_load(f)
        self.__expected_data_module_id = data["drawer_feedback"]["drawer_address"]["module_id"]
        self.__expected_data_drawer_id = data["drawer_feedback"]["drawer_address"]["drawer_id"]
        self.__expected_data_drawer_is_open = data["drawer_feedback"]["drawer_is_open"]

        self.__expected_data_error_feedback_error_code = data["error_feedback"]["error_code"]
        self.__expected_data_error_feedback_error_data = data["error_feedback"]["error_data"]

        self.__num_of_led_header_received = 0
        self.__num_of_led_states_received = 0

    def check_to_can_bus_data(self):
        # Loop through the received data on the can bus topic and check if the expected data is in the received data
        for msg in self.__received_data_from_can:
            expected_data_bytes = None

            if msg.id == can_db_defines.can_id.DRAWER_UNLOCK:
                self.assertEqual(msg.dlc, can_db_defines.can_dlc.DRAWER_UNLOCK)
                expected_data_bytes = can_data_helpers.construct_can_data_drawer_unlock(
                    self.__open_drawer_msg.module_id, self.__open_drawer_msg.drawer_id
                )

            if msg.id == can_db_defines.can_id.MODULE_CONFIG:
                self.assertEqual(msg.dlc, can_db_defines.can_dlc.MODULE_CONFIG)
                expected_data_bytes = can_data_helpers.construct_can_data_module_config(
                    self.__module_config_goal.module_address.module_id,
                    self.__module_config_goal.config_id,
                    self.__module_config_goal.config_value,
                )

            if msg.id == can_db_defines.can_id.ELECTRICAL_DRAWER_MOTOR_CONTROL:
                self.assertEqual(msg.dlc, can_db_defines.can_dlc.ELECTRICAL_DRAWER_MOTOR_CONTROL)
                expected_data_bytes = can_data_helpers.construct_can_data_e_drawer_motor_control(
                    self.__electrical_drawer_motor_control_goal.module_address.module_id,
                    self.__electrical_drawer_motor_control_goal.motor_id,
                    self.__electrical_drawer_motor_control_goal.enable_motor,
                )

            if msg.id == can_db_defines.can_id.HEARTBEAT:
                self.assertEqual(msg.dlc, can_db_defines.can_dlc.HEARTBEAT)
                expected_data_bytes = can_data_helpers.construct_heartbeat_can_frame(
                    self.__heartbeat_module_id, self.__heartbeat_interval_in_ms
                )

            if msg.id == can_db_defines.can_id.LED_HEADER:
                self.assertEqual(msg.dlc, can_db_defines.can_dlc.LED_HEADER)
                if self.__num_of_led_header_received >= len(self.__num_of_led_state_msgs):
                    self.__node.get_logger().info("WARNING: Received more LED_HEADER messages than expected!")
                    continue
                expected_data_bytes = can_data_helpers.construct_can_data_led_header(
                    self.__led_cmd_msg.drawer_address.module_id,
                    int(self.__led_cmd_msg.fade_time_in_ms / 100),
                    self.__num_of_led_state_msgs,
                    self.__num_of_led_header_received,
                    self.__ack_requested[self.__num_of_led_header_received],
                )
                self.__num_of_led_header_received += 1

            if msg.id == can_db_defines.can_id.LED_STATE:
                self.assertEqual(msg.dlc, can_db_defines.can_dlc.LED_STATE)
                if self.__num_of_led_states_received >= len(self.__num_of_led_state_msgs):
                    self.__node.get_logger().info("WARNING: Received more LED_STATE messages than expected!")
                    continue
                expected_data_bytes = can_data_helpers.construct_can_data_led_state(
                    self.__led_cmd_msg.drawer_address.module_id,
                    self.__led_states[self.__num_of_led_states_received],
                    self.__num_of_led_state_msgs[self.__num_of_led_states_received],
                    self.__ack_requested[self.__num_of_led_states_received],
                )
                self.__num_of_led_states_received += 1

            if expected_data_bytes is not None:
                self.__node.get_logger().info('Checking data bytes for can_id: "%s"' % msg.id)
                self.__node.get_logger().info(f'Expected data bytes: {[int(byte) for byte in expected_data_bytes]}')
                self.__node.get_logger().info(f'Received data bytes: {msg.data}')
                # Compare each data byte
                for i, byte in enumerate(msg.data):
                    self.assertEqual(byte, expected_data_bytes[i])

    def test_dut_output(self):
        # Create an executor and add to_can_node to spin it in separate thread to receive data from the can bus and proceed with the test
        self.__executor = rclpy.executors.SingleThreadedExecutor()
        self.__executor.add_node(self.__can_node)
        self.__spin_thread = threading.Thread(target=self.__executor.spin, daemon=True)
        self.__spin_thread.start()

        self.get_expected_results()

        self.publish_data_to_dut()

        try:
            self.__node.get_logger().info("Starting to compare received data with expected data!")
            while not self.__received_drawer_feedback_topic:
                rclpy.spin_once(self.__node, timeout_sec=0.1)
            self.assertTrue(self.__received_drawer_feedback_topic)

            while not self.__received_heartbeat_topic:
                rclpy.spin_once(self.__node, timeout_sec=0.1)
            self.assertTrue(self.__received_heartbeat_topic)
            self.assertEqual(self.__received_data_heartbeat_id, self.__heartbeat_module_id)
            self.assertEqual(self.__received_data_interval_in_ms, self.__heartbeat_interval_in_ms)
            self.assertNotEqual(self.__received_data_stamp.sec, 0)
            self.assertNotEqual(self.__received_data_stamp.nanosec, 0)
            self.__node.get_logger().info("Finished checking received data from heartbeat!")

            # test actual output for expected output
            self.assertEqual(
                self.__received_data_drawer_is_open_module_id,
                self.__expected_data_module_id,
            )
            self.assertEqual(
                self.__received_data_drawer_is_open_drawer_id,
                self.__expected_data_drawer_id,
            )
            self.assertEqual(
                self.__received_data_drawer_is_open_drawer_is_open,
                self.__expected_data_drawer_is_open,
            )
            self.__node.get_logger().info("Finished checking received data from drawer_is_open!")

            while not self.__received_drawer_error_feedback_topic:
                rclpy.spin_once(self.__node, timeout_sec=0.1)

            self.assertEqual(
                self.__received_data_error_feedback_error_code,
                self.__expected_data_error_feedback_error_code,
            )

            for i in range(len(self.__expected_data_error_feedback_error_data)):
                self.assertEqual(
                    self.__received_data_error_feedback_error_data[i],
                    self.__expected_data_error_feedback_error_data[i],
                )
            self.__node.get_logger().info("Finished checking received data from robast_error!")

            # Check if action calls were successful
            self.assertTrue(self.__setting_module_config_succeeded)
            self.assertTrue(self.__setting_electrical_drawer_motor_control_succeeded)

            self.check_to_can_bus_data()

            self.assertEqual(self.__num_of_led_header_received, len(self.__num_of_led_state_msgs))
            self.assertEqual(self.__num_of_led_states_received, len(self.__num_of_led_state_msgs))

        finally:
            self.__node.destroy_publisher(self.__can_in_publisher)
            self.__node.destroy_publisher(self.__open_drawer_publisher)
            self.__node.destroy_publisher(self.__led_cmd_publisher)
            self.__node.destroy_subscription(self.__drawer_feedback_subscriber)
            self.__node.destroy_subscription(self.__robast_error_subscriber)
            self.__node.destroy_subscription(self.__heartbeat_subscriber)
            self.__node.destroy_client(self.__module_config_service_client)
            self.__node.destroy_client(self.__electrical_drawer_motor_control_service_client)
            self.__can_node.destroy_subscription(self.__to_can_bus_subscriber)

            self.__executor.shutdown()  # Shutdown the executor
            self.__spin_thread.join()  # Wait for the thread to finish

            self.__can_node.destroy_node()
            self.__node.destroy_node()
