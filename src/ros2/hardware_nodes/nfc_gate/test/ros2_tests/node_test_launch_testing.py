import asyncio
from lib2to3.pgen2.token import EQUAL
import time
import unittest
import inspect
import yaml

import os
import ament_index_python

import rclpy
from rclpy.node import Node

import launch
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.util
from rclpy.action import ActionClient

import pytest
from communication_interfaces.action import AuthenticateUser

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
        name='nfc_gate',
        parameters=[{"key":input_data['nfc']['card_content']}]
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
        self.node = rclpy.create_node('nfc_gate_tester')

    def tearDown(self):
        self.node.destroy_node()

    def goal_response_callback(self, future):
        result = future.result().result
        
        self.result_key = result.permission_key_used
        self.result_error = result.error_message
        self.is_action_done=True

    def feedback_callback(self, future):
        future.feedback
        

    def test_dut_output(self, dut, proc_output):
         
        # Get current functionname
        frame = inspect.currentframe()
        function_name = inspect.getframeinfo(frame).function
        
        self.is_action_done=False
        # create action massage
        test_goal_msg = AuthenticateUser.Goal()
        test_goal_msg.permission_keys = [input_data['nfc']['authorised_user_code']]

        #call the Service to test
        self._action_client = ActionClient(self.node, AuthenticateUser, 'authenticate_user')
        self._action_client.wait_for_server()
        self._action_client.send_goal_async(test_goal_msg, feedback_callback=self.feedback_callback).add_done_callback( lambda future: future.result().get_result_async().add_done_callback(self.goal_response_callback))
       
        # Read data of expected result
        EXPECTED_DATA_PATH = os.path.join(
        ament_index_python.get_package_prefix('nfc_gate'),
        'lib/nfc_gate',
        'node_test_expected_data.yaml'
        )

        with open(EXPECTED_DATA_PATH) as f:
            data = yaml.safe_load(f)
        expected_result = data['nfc']['authentification_code']
        expected_error = data['nfc']['authentification_error_message']

        try:
            
            end_time = time.time() + 1
            while not self.is_action_done:
                rclpy.spin_once(self.node, timeout_sec=10.1)
            
            self.assertEqual(self.result_key, expected_result)
            self.assertEqual(self.result_error, expected_error)

        finally:
            self.node.destroy_service(self._action_client) 
            