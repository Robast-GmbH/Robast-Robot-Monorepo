
import os
import unittest
from std_msgs.msg import Int64, Bool
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
        ament_index_python.get_package_prefix('nfc_bridge'),
        'lib/nfc_bridge',
        'node_test_input_data.yaml'
    )

    global input_data
    with open(INPUT_DATA_PATH) as f:
        input_data = yaml.safe_load(f)

    dut = Node(
        package='nfc_bridge',
        executable='nfc_bridge_test',
        name='nfc_bridge_ros2_test',
        parameters=[{
                        "key": input_data['nfc']['card_content'],
                        "User1_key": input_data['nfc']['card_content'],
                        "User1_name": input_data['nfc']['authorised_user'],
                        "User1_id": input_data['nfc']['authorised_user_id']
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
        self.node = rclpy.create_node('nfc_bridge_tester')
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=2
        )

    def tearDown(self):
        self.node.destroy_node()

    def test_read_nfc(self, dut, proc_output):
        self.start_subscriber()
        self.nfc_reader_on_off(True)
        while not self.received_nfc_status_topic:
            rclpy.spin_once(self.node, timeout_sec=99)
        self.nfc_reader_on_off(False)

        # Read data of expected result
        EXPECTED_DATA_PATH = os.path.join(
            ament_index_python.get_package_prefix('nfc_bridge'),
            'lib/nfc_bridge',
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
            Int64,
            '/authenticated_user',
            self.subscriber_callback,
            qos_profile=self.qos_profile
        )

    def nfc_reader_on_off(self, on):
        self.NFC_toogler_publischer = self.node.create_publisher(
            Bool,
            "/nfc_switch",
            self.qos_profile)
        msg = Bool()
        msg.data = on
        self.NFC_toogler_publischer.publish(msg)

    def subscriber_callback(self, msg):
        self.recived_chatter_message = msg.data
        self.received_nfc_status_topic = True
        print("data_recived"+str(msg.data))
