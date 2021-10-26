import os
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from std_srvs.srv import SetBool

import random



class WaypointCreator(Node):

        def __init__(self):
                super().__init__('waypoint_creator')

                self.declare_parameter('num_of_waypoints', 9)
                num_of_waypoints = self.get_parameter('num_of_waypoints').get_parameter_value().integer_value
                self.get_logger().info('Number of waypoints to be created: %d' % num_of_waypoints)

                self.trigger_map_update_srv_client = self.create_client(SetBool, 'trigger_robast_map_publishing')
                # Check if the a service is available  
                while not self.trigger_map_update_srv_client.wait_for_service(timeout_sec=1.0):
                        self.get_logger().info('Trigger_robast_map_publishing Service is not available, waiting again...')

                map_setup = self.read_map_setup(os.path.join(get_package_share_directory('testing_tools'), 'map_setup_5OG.yaml'))             
                self.waypoints = self.create_waypoints(num_of_waypoints, map_setup)

                # Define the Timer that is responsible for triggering the robast map publish 
                self.robast_map_publish_rate = 100 # seconds between timer ticks
                self.timer = self.create_timer(self.robast_map_publish_rate, self.trigger_map_publish_callback)

                # Mind that the action server changed from /FollowWaypoints (Foxy) to /follow_waypoints (Galactic)
                self.follow_waypoints_client = ActionClient(self, FollowWaypoints, 'follow_waypoints') 
                self.send_waypoints(self.waypoints)


        def trigger_map_publish_callback(self):
                self.trigger_map_update()


        def trigger_map_update(self):
                map_update_future = self.trigger_map_update_srv_client.call_async(SetBool.Request())
                map_update_future.add_done_callback(self.map_update_result_callback)
        def map_update_result_callback(self, future):
                is_map_update_successfull = future.result().success
                self.get_logger().info('Is map update successfull? {0}'.format(is_map_update_successfull))


        def create_waypoints(self, num_of_waypoints, map_setup):
                waypoints = []
                random.seed(2) # seed random number generator
                number_of_rooms = len(map_setup['rooms'])
                for i in range(1, num_of_waypoints+1):
                        room_number = random.randint(1,number_of_rooms)
                        self.get_logger().info(('Adding room ' + str(room_number) + ' as waypoint number ' + str(i) + ' to waypoints:'))
                        waypoints.append(self.get_nav_goal_pose(room_number, map_setup))
                return waypoints


        def get_nav_goal_pose(self, nav_goal_room_number, map_setup):
                nav_goal = PoseStamped()
                nav_goal.header.frame_id = 'map'
                nav_goal.header.stamp = self.get_clock().now().to_msg()
                x = map_setup['rooms'][nav_goal_room_number]['center point']['x']
                y = map_setup['rooms'][nav_goal_room_number]['center point']['y']
                # Add a small random number to the x and y value to ensure the goal is not always the same
                nav_goal.pose.position.x = x + random.uniform(-1.0, 1.0)
                nav_goal.pose.position.y = - y + random.uniform(-1.0, 1.0)
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

                # The send_goal_future instance completes when the goal request has been accepted or rejected.
                # Therefore the spinning runs just a short time until the request has been accepted or rejected.
                rclpy.spin_until_future_complete(self, send_goal_future)

                # The result of the send_goal_future is set to a ClientGoalHandle when receipt of the goal is acknowledged by an action server
                client_goal_handle = send_goal_future.result()

                if not client_goal_handle.accepted:
                        self.get_logger().error('Following ' + str(len(waypoints)) + ' waypoints request was rejected!')

                self.result_future = client_goal_handle.get_result_async()
                self.result_future.add_done_callback(self.get_result_callback)


        def get_result_callback(self, future):
                result = future.result().result
                self.get_logger().info('Result: {0}'.format(result))


        def feedback_callback(self, feedback_msg):
                self.feedback = feedback_msg.feedback

        
        def get_feedback(self):
                return self.feedback


        def is_nav_complete(self):
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

                self.get_logger().info('Goal succeeded!')
                return True
        
            
def main(args=None):
        # initialize the ROS communication
        rclpy.init(args=args)

        # declare the node constructor
        waypoint_creator = WaypointCreator()
        
        i = 0
        while not waypoint_creator.is_nav_complete():
                # Do something with the feedback
                i = i + 1
                feedback = waypoint_creator.get_feedback()
                if feedback and i % 5 == 0:
                        waypoint_creator.get_logger().info(('Executing current waypoint: ' +
                                str(feedback.current_waypoint + 1) + '/' + str(len(waypoint_creator.waypoints))))

        # shutdown the ROS communication
        rclpy.shutdown()

if __name__ == '__main__':
        main()