import rclpy
import math
import tf2_ros
from tf_transformations import euler_from_quaternion
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from geometry_msgs.msg import Twist

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

        # Subscribe to goal_status to monitor the goal result
        self.goal_status_subscriber = self.create_subscription(
            String, 'goal_status', self.goal_status_callback, 10
        )

        # Subscribe to the local costmap
        self.local_costmap_subscriber = self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', self.local_costmap_callback, 10
        )

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
            self.check_free_space()

    def local_costmap_callback(self, msg: OccupancyGrid):
        # Store the costmap data for future checks
        self.local_costmap_data = msg

    def check_free_space(self):
        # Ensure we have costmap data to perform the sector check
        if self.local_costmap_data is None:
            self.get_logger().warn("No costmap data available!")
            return

        # Get the robot's position and orientation from the TF buffer
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.robot_x = transform.transform.translation.x
            self.robot_y = transform.transform.translation.y
            quaternion = (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            )
            _, _, self.yaw = euler_from_quaternion(quaternion)
        except Exception as e:
            self.get_logger().warn(f"Could not get robot transform: {e}")
            return

        # Check if the sector in front of the robot has enough free space
        free_space_sufficient = self.perform_sector_check()

        if free_space_sufficient:
            self.get_logger().info("Free space sufficient. No action needed.")
        else:
            self.get_logger().info("Free space insufficient. Rotating to find free space...")
            self.rotate_robot()

    def perform_sector_check(self):
        """Check the sector in front of the robot using the costmap."""
        free_cells = 0
        total_cells_in_sector = 0
        resolution = self.local_costmap_data.info.resolution
        width = self.local_costmap_data.info.width
        height = self.local_costmap_data.info.height

        # Check each cell in the costmap
        for i in range(height):
            for j in range(width):
                cell_x = self.local_costmap_data.info.origin.position.x + j * resolution
                cell_y = self.local_costmap_data.info.origin.position.y + i * resolution
                dx = cell_x - self.robot_x
                dy = cell_y - self.robot_y
                distance = math.sqrt(dx * dx + dy * dy)
                angle = math.atan2(dy, dx)
                relative_angle = angle - self.yaw

                # Check if the cell is within the sector
                if distance <= self.sector_radius and abs(relative_angle) <= self.sector_angle / 2:
                    total_cells_in_sector += 1
                    index = i * width + j
                    if self.local_costmap_data.data[index] >= 0 and self.local_costmap_data.data[index] <= 70:
                        free_cells += 1

        if total_cells_in_sector > 0:
            free_space_percentage = (free_cells / total_cells_in_sector) * 100.0
        else:
            free_space_percentage = 0.0

        self.get_logger().info(f"Free space percentage: {free_space_percentage:.2f}%")

        return free_space_percentage >= self.free_space_threshold

    def rotate_robot(self):
        """Rotate the robot in place to find free space."""
        self.get_logger().info("Rotating robot to find free space...")
        twist = Twist()
        twist.angular.z = self.rotation_speed
        self.cmd_vel_publisher.publish(twist)

        # Continuously recheck free space while rotating
        if self.rotation_timer is None:
            self.rotation_timer = self.create_timer(1.0, self.recheck_free_space_during_rotation)

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

        # Cancel the rotation timer
        if self.rotation_timer is not None:
            self.rotation_timer.cancel()
            self.rotation_timer = None


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