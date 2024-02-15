import unittest

import launch
import launch_testing
import pytest
import rclpy
from launch_ros.actions import Node
import subprocess
from rcl_interfaces.msg import Log

rosout_listener_node_name = "rosout_listener"
trigger_message = "Trigger message"
node_name = "test_node"


@pytest.mark.launch_test
def generate_test_description():
    dut = Node(
        package="launch_manager",
        executable="rosout_listener",
        name=rosout_listener_node_name,
        parameters=[
            {"trigger_message": trigger_message, "node_name": node_name},
        ],
    )
    context = {"dut": dut}

    return (
        launch.LaunchDescription([dut, launch_testing.actions.ReadyToTest()]),
        context,
    )


class TestRosoutListener(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("rosout_listener_tester")
        self.rosout_publisher = self.node.create_publisher(Log, "rosout", 10)

        self.log_msg_with_trigger_message = Log()
        self.log_msg_with_trigger_message.name = node_name
        self.log_msg_with_trigger_message.msg = trigger_message

        self.log_msg_without_trigger_message = Log()
        self.log_msg_without_trigger_message.name = node_name
        self.log_msg_without_trigger_message.msg = "Some other message"

    def tearDown(self):
        self.node.destroy_node()

    def test_node_still_alive(self):
        # Publish a message to rosout
        self.rosout_publisher.publish(self.log_msg_without_trigger_message)

        # Check that the dut node is still running
        # Get the result of the `ros2 node list` command
        # The dut node should be in the list
        # Run the `ros2 node list` command and capture the output
        result = subprocess.run(
            ["ros2", "node", "list"], capture_output=True, text=True, check=True
        )

        # Split the output into a list of node names
        node_list = result.stdout.splitlines()

        # Check if the "dut" node is in the list
        self.assertTrue(("/" + rosout_listener_node_name) in node_list)

    # TODO@SomeoneSmarter:
    # The purpose of the node is, that this should trigger a 'OnProcessExit' event
    # in a launch file. This works, if you exit the node.
    # Unfortunately, I have found no way to write a test for this behavior.
    # My test works, if I destroy and shutdown the node, but then the 'OnProcessExit'
    # event is not triggered.
    # Furthermore, I don't know if exiting a node like this has any bad side effects.
    # For now it works for my launch file, so I leave it like this.

    # TODO: Find a way to test that the node exits and triggers the OnProcessExit event
    # def test_node_stopped(self):
    #     self.rosout_publisher.publish(self.log_msg_with_trigger_message)

    #     result = subprocess.run(
    #         ["ros2", "node", "list"], capture_output=True, text=True, check=True
    #     )

    #     # Split the output into a list of node names
    #     node_list = result.stdout.splitlines()

    #     # Check if the "dut" node is in the list
    #     self.assertFalse(("/" + rosout_listener_node_name) in node_list)
