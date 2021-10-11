import os
import rclpy
import yaml
import numpy as np
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints



class WaypointCreator(Node):

        def __init__(self):
                # Here we have the class constructor
                # call super() in the constructor in order to initialize the Node object
                # the parameter we pass is the node name
                super().__init__('waypoint_creator')

                self.declare_parameter('num_of_waypoints', '9')
                num_of_waypoints = self.get_parameter('num_of_waypoints').get_parameter_value().integer_value
                self.get_logger().info('Number of waypoints to be created: %d' % num_of_waypoints)

                map_setup = self.read_map_setup(os.path.join(get_package_share_directory('testing_tools'), 'map_setup_5OG.yaml'))             
                self.waypoints = self.create_waypoints(num_of_waypoints, map_setup)

                self.follow_waypoints_client = ActionClient(self, FollowWaypoints, 'FollowWaypoints')
                self.send_waypoints(self.waypoints)


        def create_waypoints(self, num_of_waypoints, map_setup):
                waypoints = []
                for i in range(1, num_of_waypoints+1):
                        # TODO: create nav goal pose random
                        self.get_logger().info('Adding goal number %d to waypoints:' % i)
                        waypoints.append(self.get_nav_goal_pose(i, map_setup))
                return waypoints


        def get_nav_goal_pose(self, nav_goal_room_number, map_setup):
                nav_goal = PoseStamped()
                nav_goal.header.frame_id = 'map'
                nav_goal.header.stamp = self.get_clock().now().to_msg()
                x = map_setup['rooms'][nav_goal_room_number]['center point']['x']
                y = map_setup['rooms'][nav_goal_room_number]['center point']['y']
                nav_goal.pose.position.x = x
                nav_goal.pose.position.y = - y
                self.get_logger().info('-> Goal X-Coordinate: %s' % nav_goal.pose.position.x)
                self.get_logger().info('-> Goal Y-Coordinate: %s' % nav_goal.pose.position.y)
                return nav_goal


        def read_map_setup(self, filename):
                with open(filename, 'r') as file:
                        map_setup = yaml.load(file, Loader=yaml.FullLoader)
                return map_setup
                

        def send_waypoints(self, waypoints):
                waypoints_goal = FollowWaypoints.Goal()
                waypoints_goal.poses = waypoints
                self.get_logger().info('Waiting for FollowWaypoints Server ...')
                self.follow_waypoints_client.wait_for_server()
                self.get_logger().info('Sending FollowWaypoints Goal: {0}'.format(waypoints_goal))
                send_goal_future = self.follow_waypoints_client.send_goal_async(waypoints_goal, feedback_callback=self.feedback_callback)

                rclpy.spin_until_future_complete(self, send_goal_future)

                self.goal_handle = send_goal_future.result()

                if not self.goal_handle.accepted:
                        self.error('Following ' + str(len(waypoints)) + ' waypoints request was rejected!')

                self.result_future = self.goal_handle.get_result_async()


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
                rclpy.shutdown()


        def feedback_callback(self, feedback_msg):
                self.feedback = feedback_msg.feedback
                #self.get_logger().info('Current Waypoint: {0}'.format(feedback.current_waypoint))
        
        def getFeedback(self):
                return self.feedback

        def isNavComplete(self):
                if not self.result_future:
                        # task was cancelled or completed
                        return True
                rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
                if self.result_future.result():
                        self.status = self.result_future.result().status
                        if self.status != GoalStatus.STATUS_SUCCEEDED:
                                self.get_logger().info('Goal with failed with status code: {0}'.format(self.status))
                                return True
                else:
                        # Timed out, still processing, not complete yet
                        return False

                self.debug('Goal succeeded!')
                return True
        
            
def main(args=None):
        # initialize the ROS communication
        rclpy.init(args=args)

        # declare the node constructor
        waypoint_creator = WaypointCreator()
        # pause the program execution, waits for a request to kill the node (ctrl+c)
        # rclpy.spin(waypoint_creator)
        # # Explicity destroy the node
        # waypoint_creator.destroy_node()
        
        i = 0
        while not waypoint_creator.isNavComplete():
                # Do something with the feedback
                i = i + 1
                feedback = waypoint_creator.getFeedback()
                if feedback and i % 5 == 0:
                        waypoint_creator.get_logger().info(('Executing current waypoint: ' +
                                str(feedback.current_waypoint + 1) + '/' + str(len(waypoint_creator.waypoints))))

        # shutdown the ROS communication
        rclpy.shutdown()

if __name__ == '__main__':
        main()