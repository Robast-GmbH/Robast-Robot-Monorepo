import rclpy
import requests
import json
from rclpy.node import Node


from std_msgs.msg import String


class TaskConnector(Node):
    Tasks=[]
    POI=[]
    
    def __init__(self):
        super().__init__('task_connector')
        self.publisher_ = self.create_publisher(String, 'raw_task', 10)   
        self.get_logger().info("The Task Connector")

        self.subscription = self.create_subscription(
            String,
            'tracked_task',
            self.listener_callback,
            10)
        self.subscription

        #Repeted call for new tasks 
        Task_pull_intervall = 5  # seconds
        self.timer = self.create_timer(Task_pull_intervall, self.handle_tasks_callback)


    def handle_new_tasks_callback(self):
        1*2
    
    def handle_update_tasks_callback(self):
        1*2 
    
                
def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = TaskConnector()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()