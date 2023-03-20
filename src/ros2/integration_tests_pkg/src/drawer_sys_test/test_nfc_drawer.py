#! /usr/bin/env python3

import os
import sys
import time

from rcl_interfaces.srv import SetParameters
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory

from communication_interfaces.msg import DrawerStatus, DrawerLeds
from std_msgs.msg import String


import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

def create_led_msg(red, green, blue, brightness, mode) -> DrawerLeds:
    drawerleds = DrawerLeds()
    
    drawerleds.red = red
    drawerleds.green = green
    drawerleds.blue = blue
    drawerleds.brightness = brightness
    drawerleds.mode = mode
    
    return drawerleds

class NfcDrawerTest(Node):

    def __init__(self):
        super().__init__(node_name='drawer_sys_test', namespace='')
        # publsiher create_publisher<communication_interfaces::msg::DrawerLeds>(topic_name_, qos)
        qos = QoSProfile(depth=10)
        self.subscription = self.create_subscription(DrawerLeds, '/drawer_leds', self.led_callback, qos_profile=qos)
        self.__led_msg_count = 0
        self.__led_list = []
        
        time.sleep(5)
        
    def setTree(self, tree_path: str):
        #the xml readout of the leds could be realized with this, but I dont see how to use it
        self.__tree_root = ET.parse(tree_path).getroot()
        
    def add_led_element(self, element: DrawerLeds):
        self.__led_list.append(element)
            
    def compareLED(self, expected: DrawerLeds, actual: DrawerLeds):   
        if expected.red != actual.red:
            self.error_msg('LED message red incorrect')
            return False
        elif expected.green != actual.green:
            self.error_msg('LED message green incorrect')
            return False
        elif expected.blue != actual.blue:
            self.error_msg('LED message blue incorrect')
            return False
        elif expected.brightness != actual.brightness:
            self.error_msg('LED message brightness incorrect')
            return False
        elif expected.mode != actual.mode:
            self.error_msg('LED message mode incorrect')
            return False
        return True
    
    def led_callback(self, msg):
        if self.compareLED(self.__led_list[self.__led_msg_count], msg):
            self.info_msg('LED message correct')
        else:
            self.error_msg('LED message incorrect')
        self.__led_msg_count += 1
        
    def info_msg(self, msg: str):
        self.get_logger().info(msg)
        
    def warn_msg(self, msg: str):
        self.get_logger().warn(msg)

    def error_msg(self, msg: str):
        self.get_logger().error(msg)
        
    def run(self):
        self.info_msg('Running test')
        self.__led_msg_count = 0
        self.info_msg('sending nfc tag 2 to start the tree')
        qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        self.nfc_start_pub = self.create_publisher(String, '/authenticated_user', qos_profile=qos)
        self.nfc_start_pub.publish(String(data='2'))
        self.info_msg('Waiting for LED messages')
        while self.__led_msg_count < len(self.__led_list):
            rclpy.spin_once(self, timeout_sec=0.1)
        self.info_msg('All LED messages received')
        return True


def main(argv=sys.argv[1:]):
    rclpy.init()
    time.sleep(5)
    
    test = NfcDrawerTest()
    statemachine_bringup_dir = get_package_share_directory('drawer_sm')
    test.setTree(os.path.join(statemachine_bringup_dir, 'trees', 'drawer_nfc_bt.xml'))
    
    test.add_led_element(create_led_msg(0, 255, 0, 128, 1))
    test.add_led_element(create_led_msg(255, 255, 255, 128, 1))
    test.add_led_element(create_led_msg(255, 128, 0, 150, 2))
    test.add_led_element(create_led_msg(0, 155, 155, 25, 1))
    
    
    result = test.run()
    time.sleep(2)
    
    test.info_msg('Done Shutting Down.')

    if not result:
        test.info_msg('Exiting failed')
        exit(1)
    else:
        test.info_msg('Exiting passed')
        exit(0)


if __name__ == '__main__':
    main()