import rclpy
import math
import tf2_ros
from tf_transformations import euler_from_quaternion
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point

class SectorCheckNode(Node):

    def __init__(self):
        super().__init__('sector_check_node')

        # Declare and get parameters for sector checking
        self.declare_parameter('sector_angle', math.pi / 4)
        self.declare_parameter('sector_radius', 0.7)
        self.declare_parameter('free_space_threshold', 80.0)

        # Get the values of the parameters
        self.sector_angle = self.get_parameter('sector_angle').get_parameter_value().double_value
        self.sector_radius = self.get_parameter('sector_radius').get_parameter_value().double_value
        self.free_space_threshold = self.get_parameter('free_space_threshold').get_parameter_value().double_value
        self.rotation_speed = 0.5
        self.check_sector_flag = False

        # Subscribe to goal_status to monitor the goal result
        self.goal_status_subscriber = self.create_subscription(
            String, 'goal_status', self.goal_status_callback, 10
        )

        # Subscribe to the local costmap
        self.local_costmap_subscriber = self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', self.local_costmap_callback, 10
        )

        self.marker_pub = self.create_publisher(Marker, 'sector_marker', 10)

        # Publisher for commanding robot rotation
        self.cmd_vel_publisher = self.create_publisher(Twist, 'diff_drive_base_controller/cmd_vel_unstamped', 10)

        # TF listener for getting the robot's pose
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Initialize costmap and rotation tracking
        self.local_costmap_data = None
        self.is_rotating = False
        self.rotation_timer = None

    def goal_status_callback(self, msg: String):
        # Check if the goal status is "SUCCEEDED"
        if msg.data == "SUCCEEDED":
            self.get_logger().info("Goal succeeded! Checking sector for free space.")
            self.check_sector_flag = True

    def local_costmap_callback(self, msg: OccupancyGrid):
        # Store the costmap data for future checks
        self.local_costmap_data = msg
        if self.check_sector_flag:
            self.check_free_space()

    def check_free_space(self):
        try:
            
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())

            # Extract robot's position (x, y)
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y

            # Extract robot's orientation (quaternion) and convert to yaw
            quaternion = (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            )
            roll, pitch, yaw = euler_from_quaternion(quaternion)  # Get yaw angle (heading)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn('Could not get robot pose from tf')
            return

        #costmap = msg
        free_cells = 0
        total_cells_in_sector = 0

        # Get costmap metadata
        resolution = self.local_costmap_data.info.resolution
        width = self.local_costmap_data.info.width
        height = self.local_costmap_data.info.height

        # Create RViz marker for visualizing the sector
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # Line width
        marker.color.a = 1.0
        marker.color.r = 1.0  # Red color

        # Marker lifetime (using the correct Duration type from builtin_interfaces)
        marker.lifetime = Duration(sec=1)  # Set duration to 1 second so it's refreshed periodically

        # Compute the points that define the sector
        for i in range(height):
            for j in range(width):
                # Compute the world coordinates of each cell
                cell_x = self.local_costmap_data.info.origin.position.x + j * resolution
                cell_y = self.local_costmap_data.info.origin.position.y + i * resolution

                # Convert cell coordinates to polar coordinates (relative to robot)
                dx = cell_x - robot_x
                dy = cell_y - robot_y
                distance = math.sqrt(dx * dx + dy * dy)
                angle = math.atan2(dy, dx)

                # Adjust the angle by the robot's yaw to rotate the sector with the robot
                relative_angle = angle - yaw

                # Check if the cell is within the sector
                if distance <= self.sector_radius and abs(relative_angle) <= self.sector_angle / 2:
                    total_cells_in_sector += 1

                    # Check if the cell is free space (0 in the costmap)
                    index = i * width + j
                    if self.local_costmap_data.data[index] >= 0 and self.local_costmap_data.data[index] <= 50:
                        free_cells += 1

                    # Add points of the sector to visualize in RViz
                    point = Point()
                    point.x = cell_x
                    point.y = cell_y
                    point.z = 0.0
                    marker.points.append(point)

        # Publish the marker for visualization
        self.marker_pub.publish(marker)

        # Calculate the percentage of free space in the sector
        free_space_percentage = 0.0
        if total_cells_in_sector > 0:
            free_space_percentage = (free_cells / total_cells_in_sector) * 100.0

        self.get_logger().info(f"Free space in sector: {free_space_percentage:.2f}%")

        # Check if the free space percentage is below the threshold
        if free_space_percentage < self.free_space_threshold:
            self.get_logger().info(f"Free space below threshold ({self.free_space_threshold}%), rotating robot...")
            self.rotate_robot()
        else:
            self.get_logger().info("Free space above threshold, stopping rotation.")
            self.stop_robot()
            self.check_sector_flag = False

    def rotate_robot(self):
        """Rotate the robot in place to find free space."""
        self.get_logger().info("Rotating robot to find free space...")
        twist = Twist()
        twist.angular.z = self.rotation_speed
        self.cmd_vel_publisher.publish(twist)

        # # Continuously recheck free space while rotating
        # if self.rotation_timer is None:
        #     self.rotation_timer = self.create_timer(1.0, self.recheck_free_space_during_rotation)

    def recheck_free_space_during_rotation(self):
        """Recheck sector while robot is rotating."""
        self.get_logger().info("Rechecking free space during rotation...")
        if self.perform_sector_check():
            self.get_logger().info("Free space found during rotation. Stopping robot...")
            self.stop_robot()
        else:
            self.rotate_robot()

    def stop_robot(self):
        """Stop the robot's rotation."""
        twist = Twist()
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

        # # Cancel the rotation timer
        # if self.rotation_timer is not None:
        #     self.rotation_timer.cancel()
        #     self.rotation_timer = None


def main(args=None):
    rclpy.init(args=args)

    node = SectorCheckNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()