
import os
import unittest
from std_msgs.msg import String
import ament_index_python
import launch
import launch_testing
import launch_testing.util
import pytest
import rclpy
import yaml
from launch_ros.actions import Node
from rclpy.qos import (DurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)


@pytest.mark.launch_test
def generate_test_description():
    # Read input data that is send to dut
    INPUT_DATA_PATH = os.path.join(
        ament_index_python.get_package_prefix('nfc_gate'),
        'lib/nfc_gate',
        'node_test_input_data.yaml'
    )

    global input_data
    with open(INPUT_DATA_PATH) as f:
        input_data = yaml.safe_load(f)

    dut = Node(
        package='nfc_gate',
        executable='nfc_gate_test',
        name='nfc_gate_ros2_test',
        parameters=[{
                        "key": input_data['nfc']['card_content'],
                        "User1_key": input_data['nfc']['card_content'],
                        "User1_name": input_data['nfc']['authorised_user']
                    }]
    )
    context = {'dut': dut}

    return (launch.LaunchDescription([
        dut,
        launch_testing.actions.ReadyToTest()]
        ), context
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
        self.node = rclpy.create_node('nfc_gate_tester')
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=2
        )
        self.start_subscriber()

    def tearDown(self):
        self.node.destroy_node()

    def test_dut_output(self, dut, proc_output):
        while not self.received_nfc_status_topic:
            rclpy.spin_once(self.node, timeout_sec=99)

        # Read data of expected result
        EXPECTED_DATA_PATH = os.path.join(
            ament_index_python.get_package_prefix('nfc_gate'),
            'lib/nfc_gate',
            'node_test_expected_data.yaml'
        )
        with open(EXPECTED_DATA_PATH) as f:
            data = yaml.safe_load(f)
        expected_result = data['nfc']['authorised_user']
        self.assertEqual(self.recived_chatter_message, expected_result)

    def start_subscriber(self):
        self.received_nfc_status_topic = False
        # Create a subscriber
        self.subscription = self.node.create_subscription(
            String,
            '/authenticate_user',
            self.subscriber_callback,
            qos_profile=self.qos_profile
        )

    def subscriber_callback(self, msg):
        self.recived_chatter_message = msg.data
        self.received_nfc_status_topic = True
