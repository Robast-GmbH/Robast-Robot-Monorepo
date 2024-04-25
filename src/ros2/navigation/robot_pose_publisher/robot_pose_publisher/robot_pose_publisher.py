from geometry_msgs.msg import Point

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class FrameListener(Node):

    def __init__(self):
        super().__init__("robot_pose_publisher")

        # Declare and acquire `target_frame` parameter
        self.target_frame = (
            self.declare_parameter("target_frame", "robot_base_link")
            .get_parameter_value()
            .string_value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self,spin_thread=True)

        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)

        self.robot_position_publisher = self.create_publisher(
            Point, "robot_position", 10
        )

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = self.target_frame
        to_frame_rel = "map"

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel, from_frame_rel, rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().info(
                f"Could not transform {to_frame_rel} to {from_frame_rel}: {ex}"
            )
            return
        print(
            t.transform.translation.x, t.transform.translation.y, t.transform.rotation.z
        )

        # Publish the robot position
        msg = Point()
        msg.x = t.transform.translation.x
        msg.y = t.transform.translation.y
        msg.z = t.transform.rotation.z
        self.robot_position_publisher.publish(msg)


def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
