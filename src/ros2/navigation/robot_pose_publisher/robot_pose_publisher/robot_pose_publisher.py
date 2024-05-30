from geometry_msgs.msg import Point

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import numpy as np


class RobotPosePublisher(Node):
    def __init__(self):
        super().__init__("robot_pose_publisher")

        self.declare_parameter("use_robot_source_frame", True)
        self.__use_robot_source_frame = (
            self.get_parameter("use_robot_source_frame")
            .get_parameter_value()
            .bool_value
        )
        self.__source_frame = (
            "robot/base_link" if self.__use_robot_source_frame else "base_link"
        )
        self.__target_frame = "map"

        self._tf_buffer = Buffer()
        self.__tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

        # Call on_timer function every second
        self.__timer = self.create_timer(1.0, self.__on_timer)

        self.__robot_position_publisher = self.create_publisher(
            Point, "robot_position", 10
        )

    def __get_yaw_from_quaternion(self, qz, qw):
        """
        Convert a quaternion to a yaw angle.

        Input
            :param qz: The z component of the quaternion.
            :param qw: The w component of the quaternion.

        Output
            :return yaw: The yaw (rotation around z-axis) angle in radians.
        """
        return 2 * np.arctan2(qz, qw)

    def __on_timer(self):
        try:
            t = self._tf_buffer.lookup_transform(
                self.__target_frame, self.__source_frame, rclpy.time.Time()
            )

            self.get_logger().debug(
                f"Publishing robot position: {t.transform.translation.x}, {t.transform.translation.y}, {t.transform.rotation.z}"
            )
            # Publish the robot position
            msg = Point()
            msg.x = t.transform.translation.x
            msg.y = t.transform.translation.y
            msg.z = self.__get_yaw_from_quaternion(
                t.transform.rotation.z, t.transform.rotation.w
            )
            self.__robot_position_publisher.publish(msg)

        except TransformException as ex:
            self.get_logger().info(
                f"Could not transform {self.__source_frame} to {self.__target_frame}: {ex}"
            )


def main():
    rclpy.init()
    node = RobotPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
