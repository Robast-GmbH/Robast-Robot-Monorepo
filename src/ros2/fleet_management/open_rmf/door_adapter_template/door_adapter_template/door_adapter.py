import sys
import yaml
import argparse

import time
import threading

import rclpy
from door_adapter_template.DoorClientAPI import DoorClientAPI
from rclpy.node import Node
from rmf_door_msgs.msg import DoorRequest, DoorState, DoorMode

###############################################################################


class DoorAdapter(Node):
    def __init__(self, config_yaml):
        super().__init__("door_adapter")
        self.get_logger().info("Starting door adapter...")

        # Get value from config file
        self.__door_name = config_yaml["door"]["name"]
        self.__door_close_feature = config_yaml["door"]["door_close_feature"]
        self.__door_signal_period_in_sec = config_yaml["door"][
            "door_signal_period_in_sec"
        ]
        self.__door_state_publish_period_in_sec = config_yaml["door_publisher"][
            "door_state_publish_period_in_sec"
        ]

        url = config_yaml["door"]["api_endpoint"]

        door_pub = config_yaml["door_publisher"]
        door_sub = config_yaml["door_subscriber"]

        self.__api = DoorClientAPI(url)

        assert self.__api.connected, "Unable to establish connection with door"

        # default door state - closed mode
        self.__door_mode = DoorMode.MODE_CLOSED
        # open door flag
        self.__open_door = False
        self.__check_status = False

        self.__door_states_pub = self.create_publisher(
            DoorState, door_pub["topic_name"], 10
        )

        self.create_subscription(
            DoorRequest, door_sub["topic_name"], self.__door_request_cb, 10
        )

        self.periodic_timer = self.create_timer(
            self.__door_state_publish_period_in_sec, self.__time_cb
        )

    def __door_open_command_request(self):
        # assume API doesn't have close door API
        # Once the door command is posted to the door API,
        # the door will be opened and then close after 5 secs
        while self.__open_door:
            success = self.__api.open_door()
            if success:
                self.get_logger().info(
                    f"Request to open door [{self.__door_name}] is successful"
                )
            else:
                self.get_logger().warning(
                    f"Request to open door [{self.__door_name}] is unsuccessful"
                )
            time.sleep(self.__door_signal_period_in_sec)

    def __time_cb(self):
        if self.__check_status:
            self.__door_mode = self.__api.get_mode()
            # when door request is to close door and the door state is close
            # will assume the door state is close until next door open request
            # This implement to reduce the number of API called
            if self.__door_mode == DoorMode.MODE_CLOSED and not self.__open_door:
                self.__check_status = False
        state_msg = DoorState()
        state_msg.door_time = self.get_clock().now().to_msg()

        # publish states of the door
        state_msg.door_name = self.__door_name
        state_msg.current_mode.value = self.__door_mode
        self.__door_states_pub.publish(state_msg)

    def __door_request_cb(self, msg: DoorRequest):
        # when door node receive open request, the door adapter will send open command to API
        # If door node receive close request, the door adapter will stop sending open command to API
        # check DoorRequest msg whether the door name of the request is same as the current door. If not, ignore the request
        if msg.door_name == self.__door_name:
            self.get_logger().info(
                f"Door mode [{msg.requested_mode.value}] requested by {msg.requester_id}"
            )
            if msg.requested_mode.value == DoorMode.MODE_OPEN:
                # open door implementation
                self.__open_door = True
                self.__check_status = True
                if self.__door_close_feature:
                    self.__api.open_door()
                else:
                    t = threading.Thread(target=self.__door_open_command_request)
                    t.start()
            elif msg.requested_mode.value == DoorMode.MODE_CLOSED:
                # close door implementation
                self.__open_door = False
                self.get_logger().info("Close Command to door received")
                if self.__door_close_feature:
                    self.__api.close_door()
            else:
                self.get_logger().error("Invalid door mode requested. Ignoring...")


###############################################################################


def main(argv=sys.argv):
    rclpy.init(args=argv)

    args_without_ros = rclpy.utilities.remove_ros_args(argv)
    parser = argparse.ArgumentParser(
        prog="door_adapter", description="Configure and spin up door adapter for door "
    )
    parser.add_argument(
        "-c",
        "--config_file",
        type=str,
        required=True,
        help="Path to the config.yaml file for this door adapter",
    )
    args = parser.parse_args(args_without_ros[1:])
    config_path = args.config_file

    # Load config and nav graph yamls
    with open(config_path, "r") as f:
        config_yaml = yaml.safe_load(f)

    door_adapter = DoorAdapter(config_yaml)
    rclpy.spin(door_adapter)

    door_adapter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
