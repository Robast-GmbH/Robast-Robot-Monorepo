import rclpy
import requests
import time
import json
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from datetime import datetime


class SimpleFleetmanagement(Node):

    def __init__(self):
        super().__init__('simple_fleetmanagement')

        # the interval in which the task get pulled from backend interface.
        self.task_pull_intervall = 5  # seconds
        # the radius in which the robot defines the goal reached.
        self.goal_reach_epsilon = 0.4  # meter
        # the time which is acceptable for no new feedback message to be sent after that the navigation will be reset.
        self.error_reset_time = 20  # seconds

        # order queue is a array that contains tuples with (order_id, nav_goal_within_room nav_goal_door_bell)
        self.order_queue = []
        # waypoint_queue contains the nav goals of the door_bell and the pose within the room for each order
        self.waypoint_queue = []
        self.last_feedback = datetime.now()

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.last_feedback = datetime.now()

        # Repeted call for new tasks
        self.get_logger().info("The simple fleetmanagement is running")
        self.timer = self.create_timer(self.task_pull_intervall, self.handle_tasks_callback)

    def handle_tasks_callback(self):
        is_queue_empty_before_getting = len(self.order_queue) == 0
        self.get_tasks()

        time_diff = datetime.now() - self.last_feedback

        if (is_queue_empty_before_getting or time_diff.total_seconds() > self.error_reset_time) and len(self.order_queue) > 0:
            order_id = self.order_queue[0][0]
            self.create_waypoints_for_order(order_id)

            self.send_nav_goal()
            #self.get_logger().info("Last callback: "+str(self.last_feedback ))
            #self.get_logger().info("duration : "+str(time_diff.total_seconds() ))

    def create_waypoints_for_task(self, order_id):
        nav_goal_within_room = self.order_queue[0][1]
        nav_goal_door_bell = self.order_queue[0][2]

    def send_nav_goal(self):
        if len(self.order_queue) > 0:

            self.get_nav_goal(nav_goal_within_room)

    def get_nav_goal(self, goal):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = float(nav_goal_within_room["x"]) / 100
        goal_msg.pose.pose.position.y = float(nav_goal_within_room["y"]) / -100
        self.get_logger().info('Navigating to goal: ' + str(goal_msg.pose.pose.position.x) + ' ' +
                               str(goal_msg.pose.pose.position.y) + ' for order ' + str(order_id))

        def order_feedback_callback(feedback_msg): return self.feedback_callback(order_id, feedback_msg)
        self.send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg, feedback_callback=order_feedback_callback)

        # goal_response_callback is called before the task is finished (don't know why).
        # So better use the feedback topic abd check distance_remaining
        def order_goal_response_callback(a): return self.goal_response_callback(order_id, a)
        # self.send_goal_future.add_done_callback(order_goal_response_callback)

    def feedback_callback(self, order_id, feedback_msg):
        feedback = feedback_msg.feedback
        self.last_feedback = datetime.now()
        if (feedback.distance_remaining < self.goal_reach_epsilon):
            self.set_order_to_finished(order_id)
        # else:
        #    self.get_logger().debug('Received feedback: {0}'.format(feedback.distance_remaining))

    def goal_response_callback(self, order_id, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        def order_get_result_callback(a): return self.get_result_callback(order_id, a)
        self._get_result_future.add_done_callback(order_get_result_callback)

    def get_result_callback(self, order_id, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        self.set_order_to_finished(order_id)

    def set_order_to_finished(self, order_id):
        # Send update request with finished=true
        response = self.update_order(order_id, "finished", True)

        if response.status_code == 200:
            initial_size = len(self.order_queue)
            # Remove finished task from queue
            self.order_queue = list(filter(lambda order_item: order_item[0] != order_id, self.order_queue))

            if len(self.order_queue) < initial_size:
                self.get_logger().info('Order with order_id {0} is finished!'.format(order_id))
                # Send next nav_goal to robot
                self.send_nav_goal()
        else:
            self.get_logger().info('Server does not respond. Order ' + str(order_id) + ' could not be finished.')

    def update_order(self, order_id, property, value):
        url = "http://backend:8000/orders/" + str(order_id)
        self.checkConnection(url)

        order_json = self.get_order(url)
        order_json[property] = value

        response = requests.put(url, data=json.dumps(order_json))
        return response

    def get_order(self, url):
        self.checkConnection(url)
        response_get_order = requests.get(url)
        response_json = json.loads(response_get_order.text)
        return response_json

    def does_queue_contains_order(self, order_id):
        for nav_goal_by_order_id in self.order_queue:
            if order_id == nav_goal_by_order_id[0]:
                return True
        return False

    def get_tasks(self):
        api_url = "http://backend:8000/orders/"
        self.checkConnection(api_url)
        response = requests.get(api_url)
        if(response.status_code == 200):
            response_text = response.json()
            for item in response_text:
                if not item["finished"]:
                    order_id = item["id"]
                    nav_goal_within_room = item["goal"]
                    nav_goal_door_bell = item["door_bell"]
                    if not self.does_queue_contains_order(order_id):
                        self.order_queue.append((order_id, nav_goal_within_room, nav_goal_door_bell))
                        self.get_logger().info('New Task with order_id {0} was appended to queue!'.format(order_id))

    def checkConnection(self, url):
        noConnection = True
        errorSent = False

        while noConnection:
            try:
                # self.get_logger().info("test")
                response = requests.get(url)
                noConnection = False
            except requests.exceptions.RequestException:
                if not errorSent:
                    self.get_logger().info("connection to " + url + " failed.")
                    errorSent = True
                time.sleep(1)  # if fixed in use wait until

        if errorSent:
            self.get_logger().info("connection established.")
        return url


def main(args=None):
    rclpy.init(args=args)
    fleetmanagement = SimpleFleetmanagement()
    rclpy.spin(fleetmanagement)
    fleetmanagement.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
