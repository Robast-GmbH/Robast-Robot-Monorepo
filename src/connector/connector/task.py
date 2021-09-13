import rclpy
import requests
import json
from rclpy.node import Node


from std_msgs.msg import String


class TaskConnectorPublisher(Node):

    
    def __init__(self):
        super().__init__('task_connector')
        self.publisher_ = self.create_publisher(String, '/raw_task', 10)
        self.i = -1
        self.get_Task_callback()
        #Repeted call for new tasks 
        Task_pull_intervall = 5  # seconds
        self.timer = self.create_timer(Task_pull_intervall, self.get_Task_callback)
        self.i = 0

    def get_Task_callback(self):
        msg = String()
        tasks = self.get_tasks()
        
        for task in tasks:
            message = ",".join(task)
            msg.data = message
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s" "%s"' % ( message, self.i)  )
        self.i += 1
    
    def get_tasks(self):
        
        api_url = "https://reqbin.com/echo/get/json/page/2"
        response = requests.get(api_url)
        response_text= response.json()
        tasks= []
        for item in response_text["items"]:
            task=[]
            task.append(str(item["id"]))
            task.append(item["name"])
            task.append(str(item["price"]))
            tasks.append(task)
        return tasks
                
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = TaskConnectorPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()