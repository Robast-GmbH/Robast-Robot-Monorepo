import rclpy
from rclpy import subscription
from rclpy.node import Node

from std_msgs.msg import String

class RobotBroker(Node):

        aktiveRobots=[]

        def __init__(self):
         super().__init__('robot_broker')

        def subscribe_to_robots(self):
                robotIdentifier=[]#ToDo
                topic_list,  node = get_topic_list()
                for info in topic_list:
                        targetTopic=""#ToDo
                        if robotIdentifier in info[0] and  info[0].split('/').end == targetTopic:
                                namespace = next(x for x in info[0].split('/') if robotIdentifier in x)
                                if namespace not in self.aktiveRobots:
                                        self.aktiveRobots.append(namespace)
                                self.subscribe(info[0],"",)#Todo Which message / define callbacks


        def subscribe(self,topic,messageType ,callback): 
                self.subscription = self.create_subscription(
                	messageType,
                        topic,
                        callback,
                        10)
                self.subscription # prevent unused variable warning
        
        def publish(self,msg):
                # msg = String()
                # msg.data = 'Hello World: %d' % self.i
                self.publisher_.publish(msg)
                self.get_logger().info('Publishing: "%s"' % msg.data)
        


   
def get_topic_list():
        node_dummy = Node("ros_info")
        topic_list = node_dummy.get_topic_names_and_types()
        node_dummy.destroy_node()
        return topic_list, node_dummy



def main(args=None):
    rclpy.init(args=args)
    robot_broker = RobotBroker()
    rclpy.spin(robot_broker)
    robot_broker.subscribe_to_robots()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
