import sys
import yaml
import argparse

import time
import threading

import rclpy
from door_adapter_template.DoorClientAPI import DoorClientAPI
from rclpy.node import Node
from rclpy.time import Time
from rmf_door_msgs.msg import DoorRequest, DoorState, DoorMode

###############################################################################

class DoorAdapter(Node):
    def __init__(self,config_yaml):
        super().__init__('door_adapter')
        self.get_logger().info('Starting door adapter...')

        # Get value from config file
        self.door_name = config_yaml['door']['name']
        self.door_close_feature = config_yaml['door']['door_close_feature']
        self.door_signal_period = config_yaml['door']['door_signal_period']
        self.door_state_publish_period = config_yaml['door_publisher']['door_state_publish_period']
        
        url = config_yaml['door']['api_endpoint']
        api_key = config_yaml['door']['header_key']
        api_value = config_yaml['door']['header_value']
        door_id = config_yaml['door']['door_id']
        
        door_pub = config_yaml['door_publisher']
        door_sub = config_yaml['door_subscriber']

        self.api = DoorClientAPI(url,api_key,api_value,door_id)
       
        assert self.api.connected, "Unable to establish connection with door"
        
        # default door state - closed mode
        self.door_mode = DoorMode.MODE_CLOSED
        # open door flag
        self.open_door = False
        self.check_status = False
        
        self.door_states_pub = self.create_publisher(
            DoorState, door_pub['topic_name'], 10)

        self.door_request_sub = self.create_subscription(
            DoorRequest, door_sub['topic_name'], self.door_request_cb, 10)

        self.periodic_timer = self.create_timer(
            self.door_state_publish_period, self.time_cb)

    def door_open_command_request(self):
        # assume API doesn't have close door API
        # Once the door command is posted to the door API,
        # the door will be opened and then close after 5 secs    
        while self.open_door:
            success = self.api.open_door()
            if success:
                self.get_logger().info(f"Request to open door [{self.door_name}] is successful")
            else:
                self.get_logger().warning(f"Request to open door [{self.door_name}] is unsuccessful")
            time.sleep(self.door_signal_period)

    def time_cb(self):
        if self.check_status:
            self.door_mode = self.api.get_mode()
            # when door request is to close door and the door state is close
            # will assume the door state is close until next door open request
            # This implement to reduce the number of API called
            if self.door_mode == DoorMode.MODE_CLOSED and not self.open_door:
                self.check_status = False
        state_msg = DoorState()
        state_msg.door_time = self.get_clock().now().to_msg()

        # publish states of the door
        state_msg.door_name = self.door_name
        state_msg.current_mode.value = self.door_mode
        self.door_states_pub.publish(state_msg)

    def door_request_cb(self, msg: DoorRequest):
        # when door node receive open request, the door adapter will send open command to API
        # If door node receive close request, the door adapter will stop sending open command to API
        # check DoorRequest msg whether the door name of the request is same as the current door. If not, ignore the request
        if msg.door_name == self.door_name:
            self.get_logger().info(f"Door mode [{msg.requested_mode.value}] requested by {msg.requester_id}")
            if msg.requested_mode.value == DoorMode.MODE_OPEN:
                # open door implementation
                self.open_door = True
                self.check_status = True
                if self.door_close_feature:
                    self.api.open_door()
                else:
                    t = threading.Thread(target = self.door_open_command_request)
                    t.start()
            elif msg.requested_mode.value == DoorMode.MODE_CLOSED:
                # close door implementation
                self.open_door = False
                self.get_logger().info('Close Command to door received')
                if self.door_close_feature:
                    self.api.close_door()
            else:
                self.get_logger().error('Invalid door mode requested. Ignoring...')

###############################################################################

def main(argv=sys.argv):
    rclpy.init(args=argv)

    args_without_ros = rclpy.utilities.remove_ros_args(argv)
    parser = argparse.ArgumentParser(
        prog="door_adapter",
        description="Configure and spin up door adapter for door ")
    parser.add_argument("-c", "--config_file", type=str, required=True,
                        help="Path to the config.yaml file for this door adapter")
    args = parser.parse_args(args_without_ros[1:])
    config_path = args.config_file

    # Load config and nav graph yamls
    with open(config_path, "r") as f:
        config_yaml = yaml.safe_load(f)

    door_adapter = DoorAdapter(config_yaml)
    rclpy.spin(door_adapter)

    door_adapter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
