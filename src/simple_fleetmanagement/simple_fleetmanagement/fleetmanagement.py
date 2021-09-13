import rclpy
import requests
import json
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose


class TaskConnectorPublisher(Node):

    order_queue = []
    
    def __init__(self):
        super().__init__('simple_fleetmanagement')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.handle_tasks_callback(self)

        #Repeted call for new tasks 
        Task_pull_intervall = 5  # seconds
        self.timer = self.create_timer(Task_pull_intervall, self.handle_tasks_callback)

    def handle_tasks_callback(self):
        is_queue_empty_before_getting = len(order_queue) == 0
        self.get_tasks()
        if is_queue_empty_before_getting and len(order_queue) > 0:
            send_nav_goal()
    
    def send_nav_goal(self):
        if len(order_queue) > 0:
            nav_goal_by_order_id = order_queue.pop(0) # nav_goal_by_order_id is a tuple
            order_id = nav_goal_by_order_id[0]
            nav_goal = nav_goal_by_order_id[1]
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.pose.position.x = float(nav_goal["x"])
            goal_msg.pose.pose.position.y = float(nav_goal["y"])
            self.info('Navigating to goal: ' + str(goal_msg.pose.pose.position.x) + ' ' +
                        str(goal_msg.pose.pose.position.y) + ' for order ' + str(order_id))
            self.send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
                                                                    self.feedback_callback)
            self.send_goal_future.add_done_callback(self.goal_response_callback)

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
        self.get_logger().info('Result: {0}'.format(result.status))
        # Send next nav_goal to robot
        send_nav_goal()


    def does_queue_contains_order(self, order_id):
        for nav_goal_by_order_id in order_queue:
            if order_id in nav_goal_by_order_id[0]:
                return true
        return false
                
    
    def get_tasks(self):
        api_url = "http://localhost:8000/orders"
        response = requests.get(api_url)
        if(response.status_code == 200):
            response_text= response.json()
            for item in response_text:
                if not item["finished"]:
                    order_id = item["id"]
                    nav_goal = item["goal"]
                    if not does_queue_contains_order(order_id):
                        order_queue.append((order_id, nav_goal))

                
def main(args=None):
    rclpy.init(args=args)
    fleetmanagement = TaskConnectorPublisher()
    rclpy.spin(fleetmanagement)
    fleetmanagement.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()