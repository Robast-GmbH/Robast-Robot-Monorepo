import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

class CostmapAnalyzer(Node):
    def __init__(self):
        super().__init__('costmap_analyzer')
        # Subscribe to the global costmap topic
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/keepout_filter_mask',
            self.costmap_callback,
            10)
        self.subscription  # prevent unused variable warning

    def costmap_callback(self, msg):
        # Convert the occupancy grid data to a numpy array
        costmap_array = np.array(msg.data, dtype=np.int8)
        # Find unique values in the costmap
        unique_values = np.unique(costmap_array)
        self.get_logger().info(f'Unique values in the costmap: {unique_values}')

def main(args=None):
    rclpy.init(args=args)
    costmap_analyzer = CostmapAnalyzer()
    rclpy.spin(costmap_analyzer)
    costmap_analyzer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()