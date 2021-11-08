import os
import rclpy
import yaml
import random
import time
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import SetBool


class WaypointCreator(Node):

    def __init__(self):
        super().__init__('waypoint_creator')

        time.sleep(10)  # Sleep for some seconds to make sure gazebo and other stuff is started

        self.declare_parameter('num_of_waypoints', 9)
        num_of_waypoints = self.get_parameter('num_of_waypoints').get_parameter_value().integer_value
        self.get_logger().info('Number of waypoints to be created: %d' % num_of_waypoints)

        self.declare_parameter('random_seed', 2)
        self.random_seed = self.get_parameter('random_seed').get_parameter_value().integer_value
        self.get_logger().info('Random seed: %d' % self.random_seed)

        self.declare_parameter('random_deviation_goal_pose', 1.5)
        self.random_deviation = self.get_parameter('random_deviation_goal_pose').get_parameter_value().double_value
        self.get_logger().info('Random deviation for goal pose: %f' % self.random_deviation)

        self.trigger_map_update_srv_client = self.create_client(SetBool, 'trigger_robast_map_publishing')
        self.trigger_map_update_available = True
        # Check if the a service is available
        if not self.trigger_map_update_srv_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('Trigger_robast_map_publishing Service is not available! If slam_toolbox is used, this service should be active.')
            self.trigger_map_update_available = False

        self.goal_description_by_waypoints = dict()  # This dict is important to report which rooms have failed
        map_setup = self.read_map_setup(os.path.join(
            get_package_share_directory('testing_tools'), 'map_setup_5OG.yaml'))
        self.waypoints = self.create_waypoints(num_of_waypoints, map_setup)

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Create lists to track the failed nav goals
        self.failed_nav_goals = list()
        # Create dictionary to track the number of recoveries needed for each waypoint
        self.number_of_recoveries_by_waypoint = dict()

    def trigger_map_update(self):
        if self.trigger_map_update_available:
            map_update_future = self.trigger_map_update_srv_client.call_async(SetBool.Request())
            map_update_future.add_done_callback(self.map_update_result_callback)
            # Sleep for a second to make sure there is enough time to build a new costmap before navigation is started again
            time.sleep(1.0)

    def map_update_result_callback(self, future):
        is_map_update_successfull = future.result().success
        self.get_logger().info('Is map update successfull? {0}'.format(is_map_update_successfull))

    def create_waypoints(self, num_of_waypoints, map_setup):
        waypoints = []
        random.seed(self.random_seed)  # seed random number generator
        number_of_rooms = len(map_setup['rooms'])
        for waypoint_index in range(1, num_of_waypoints+1):
            room_number = random.randint(1, number_of_rooms)
            # Firstly add the door bell of the room as nav goal to the waypoints
            poi = 'door_bell'
            nav_goal_door_bell = self.get_nav_pose_from_map_setup(room_number, poi, waypoint_index, map_setup)
            waypoints.append(nav_goal_door_bell)
            # Secondly add the nav goal within the room itself to the waypoints
            poi = 'center_point'
            nav_goal_center_point = self.get_nav_pose_from_map_setup(room_number, poi, waypoint_index * 2, map_setup)
            waypoints.append(nav_goal_center_point)
        return waypoints

    def get_nav_pose_from_map_setup(self, room_number, point_of_interest, waypoint_index, map_setup):
        self.get_logger().info(('Adding ' + str(point_of_interest) + ' of room ' + str(room_number) +
                                ' as waypoint number ' + str(waypoint_index) + ' to waypoints:'))
        nav_goal = PoseStamped()
        nav_goal.header.frame_id = 'map'
        nav_goal.header.stamp = self.get_clock().now().to_msg()
        x = map_setup['rooms'][room_number][point_of_interest]['x']
        y = map_setup['rooms'][room_number][point_of_interest]['y']
        if point_of_interest == 'center_point':
            # Add a small random number to the x and y value to ensure the goal is not always the same within the room
            nav_goal.pose.position.x = x + random.uniform(-self.random_deviation, self.random_deviation)
            nav_goal.pose.position.y = - y + random.uniform(-self.random_deviation, self.random_deviation)
        self.get_logger().info('-> Goal X-Coordinate: %s' % nav_goal.pose.position.x)
        self.get_logger().info('-> Goal Y-Coordinate: %s' % nav_goal.pose.position.y)
        # Track which goal is represented by each waypoint
        self.goal_description_by_waypoints[waypoint_index] = str(point_of_interest) + '_room_' + str(room_number)
        return nav_goal

    def read_map_setup(self, filename):
        with open(filename, 'r') as file:
            map_setup = yaml.load(file, Loader=yaml.FullLoader)
        return map_setup

    def send_nav_goal(self, nav_goal, waypoint_counter):
        nav_to_pose_goal_msg = NavigateToPose.Goal()
        nav_to_pose_goal_msg.pose = nav_goal

        self.get_logger().info('Sending waypoint number {0} as NavigateToPose Goal: '.format(waypoint_counter))
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            nav_to_pose_goal_msg, feedback_callback=self.feedback_callback)

        # The send_goal_future instance completes when the goal request has been accepted or rejected.
        # Therefore the spinning runs just a short time until the request has been accepted or rejected.
        rclpy.spin_until_future_complete(self, send_goal_future)
        # The result of the send_goal_future is set to a ClientGoalHandle when receipt of the goal is acknowledged by an action server
        client_goal_handle = send_goal_future.result()

        if not client_goal_handle.accepted:
            self.get_logger().error('NavigateToPose request number ' + str(waypoint_counter) + ' was rejected!')

        self.result_future = client_goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))

    def feedback_callback(self, feedback_msg):
        self.feedback = feedback_msg.feedback

    def get_feedback(self):
        return self.feedback

    def is_nav_complete(self, waypoint_counter):
        if not self.result_future:
            # task was cancelled or completed
            self.get_logger().info('Goal cancelled or completed!')
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.handle_failed_nav_goal(waypoint_counter)
                return True
            elif self.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Goal succeeded!')
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

    def handle_failed_nav_goal(self, waypoint_counter):
        self.get_logger().info('Goal for waypoint {0} failed with status code: {1}! List of failed goales:'.format(
            waypoint_counter, self.status))
        goal_description = self.goal_description_by_waypoints[waypoint_counter]
        goal_position_x = self.waypoints[waypoint_counter - 1].pose.position.x
        goal_position_y = self.waypoints[waypoint_counter - 1].pose.position.y
        self.failed_nav_goals.append(
            "Waypoint {0} with the goal {1} with goal pose x = {2} and y = {3} failed!"
            .format(waypoint_counter, goal_description, goal_position_x, goal_position_y))
        for failed_nav_goal in self.failed_nav_goals:
            self.get_logger().info('{0}'.format(failed_nav_goal))

    def spin_until_nav_goal_reached(self, waypoint_counter):
        i = 0
        number_of_recoveries = 0
        while not self.is_nav_complete(waypoint_counter):
            # Do something with the feedback
            i = i + 1
            feedback = self.get_feedback()
            if feedback.number_of_recoveries > number_of_recoveries:
                number_of_recoveries = feedback.number_of_recoveries
                self.number_of_recoveries_by_waypoint[waypoint_counter] = number_of_recoveries
                self.get_logger().info('Number of recoveries for waypoint ' +
                                       str(waypoint_counter) + ' increased to ' + str(number_of_recoveries))
            if feedback and i % 50 == 0:
                self.get_logger().info('Executing current waypoint: ' +
                                       str(waypoint_counter) + '/' + str(len(self.waypoints)))


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)

    # declare the node constructor
    waypoint_creator = WaypointCreator()

    waypoint_creator.get_logger().info('Waiting for navigate_to_pose Server ...')
    waypoint_creator.nav_to_pose_client.wait_for_server()
    waypoint_counter = 0

    for nav_goal in waypoint_creator.waypoints:
        waypoint_counter += 1
        waypoint_creator.trigger_map_update()
        waypoint_creator.send_nav_goal(nav_goal, waypoint_counter)
        waypoint_creator.spin_until_nav_goal_reached(waypoint_counter)

    waypoint_creator.get_logger().info('Finished test procedure for {0} waypoints!'.format(waypoint_counter))

    write_result_log(waypoint_creator, waypoint_counter)
    waypoint_creator.get_logger().info(
        'Wrote all {0} failed waypoints to waypoint_testing_result.cache!'.format(len(waypoint_creator.failed_nav_goals)))

    # shutdown the ROS communication
    rclpy.shutdown()


def write_result_log(waypoint_creator, waypoint_counter):
    # get timestamp
    datetime_string = datetime.now().strftime("%d/%m/%Y %H:%M:%S")  # dd/mm/YY H:M:S

    # compute total number of recoveries
    total_num_of_recoveries = 0
    for num_of_recoveries in waypoint_creator.number_of_recoveries_by_waypoint.values():
        total_num_of_recoveries += num_of_recoveries

    # write the list of failed_nav_goals to txt file
    with open('waypoint_testing_result.cache', 'a+') as file:
        file.write("\n\n" + "Timestamp:" + datetime_string)
        file.write("\n" + "Finished test procedure for " +
                   str(waypoint_counter) + " waypoints! List of all " + str(len(waypoint_creator.failed_nav_goals)) + " failed waypoints:")
        for item in waypoint_creator.failed_nav_goals:
            file.write("\n%s" % item)
        file.write("\n" + "Total Number of recoveries: " + str(total_num_of_recoveries) +
                   ". List of recoveries for each waypoint.")
        file.write("\n" + "waypoint | goal_description | number_of_recoveries:")
        for waypoint, num_of_recoveries in waypoint_creator.number_of_recoveries_by_waypoint.items():
            # adjust the number of empty spaces according to the number of digits of the waypoints
            if waypoint < 10:
                file.write("\n" + str(waypoint) + "                   ")
            else:
                file.write("\n" + str(waypoint) + "                  ")
            file.write(str(waypoint_creator.goal_description_by_waypoints[waypoint - 1]) + "                  ")
            file.write(str(num_of_recoveries))


if __name__ == '__main__':
    main()
