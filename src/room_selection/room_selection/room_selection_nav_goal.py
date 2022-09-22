import os
from ament_index_python.packages import get_package_share_directory
from contextlib import nullcontext
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped
# import Quality of Service library, to set the correct profile and reliability in order to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.action import ActionClient
import yaml
from nav2_msgs.action import NavigateToPose


class RoomSelectionNavGoal(Node):

        def __init__(self):
                super().__init__('room_selection_nav_goal')
                self.map_setup = self.read_map_setup(os.path.join(get_package_share_directory('room_selection'), 'map_setup.yaml'))

                #ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {pose: {position: {x: 2.2, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
                self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

                #ros2 topic pub /room_nav_goal std_msgs/msg/Int8 "data: 10"
                self.subscriber = self.create_subscription(Int8, '/room_nav_goal', self.initiate_navigation, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        def initiate_navigation(self, msg):
                nav_goal_room_number = msg.data
                self.get_logger().info('Goal Pose is room: %s' % nav_goal_room_number)

                if len(self.map_setup['rooms']) > nav_goal_room_number:
                        nav_goal = self.get_nav_goal_pose(nav_goal_room_number)                    
                        self.send_nav_goal(nav_goal)
                else:
                        self.get_logger().info('The room %s is not within the available number of rooms!' % nav_goal_room_number)

        def get_nav_goal_pose(self, nav_goal_room_number):
                nav_goal = PoseStamped()
                x = self.map_setup['rooms'][nav_goal_room_number]['center point']['x']
                y = self.map_setup['rooms'][nav_goal_room_number]['center point']['y']
                nav_goal.pose.position.x = x/100.0
                nav_goal.pose.position.y = - y/100.0
                self.get_logger().info('Goal X-Coordinate: %s' % nav_goal.pose.position.x)
                self.get_logger().info('Goal Y-Coordinate: %s' % nav_goal.pose.position.y)
                return nav_goal


        def read_map_setup(self, filename):
                with open(filename, 'r') as file:
                        map_setup = yaml.load(file, Loader=yaml.FullLoader)
                return map_setup


        def send_nav_goal(self, nav_pose):
                nav_pose_action_goal = NavigateToPose.Goal()
                nav_pose_action_goal.pose = nav_pose
                self.action_client.wait_for_server()
                self._send_goal_future = self.action_client.send_goal_async(nav_pose_action_goal, feedback_callback=self.feedback_callback)

                self._send_goal_future.add_done_callback(self.goal_response_callback)

        def goal_response_callback(self, future):
                goal_handle = future.result()
                if not goal_handle.accepted:
                        self.get_logger().info('Goal rejected :(')
                        return

                self.get_logger().info('Goal accepted :)')

                self._get_result_future = goal_handle.get_result_async()
                self._get_result_future.add_done_callback(self.get_result_callback)
        
        def get_result_callback(self, future):
                result = future.result().result
                self.get_logger().info('Result: {0}'.format(result))
                # rclpy.shutdown()

        def feedback_callback(self, feedback_msg):
                feedback = feedback_msg.feedback
                self.get_logger().info('Received feedback: {0}'.format(feedback.distance_remaining))


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    room_selection_nav_goal = RoomSelectionNavGoal()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(room_selection_nav_goal)
    # Explicity destroy the node
    room_selection_nav_goal.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()