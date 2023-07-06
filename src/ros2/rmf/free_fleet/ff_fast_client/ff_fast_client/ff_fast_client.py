import rclpy
from std_msgs.msg import String
from rclpy.node import Node
from cyclonedds.domain import DomainParticipant
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from . import messages
from threading import Thread

class ff_fast_client(Node):

    def __init__(self):
        super().__init__('ff_fast_client')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.start_receiving_from_dds()
        self.fleet_name="ROBAST_1"
        self.name="RB1"
    
    def start_receiving_from_dds(self):
        open_drawer = Thread(target=self.dds_subscriber, args=(42, "OpenDrawerequest",messages.FreeFleetData_OpenDrawerRequest,self.ros_publish_drawer_open ), daemon=True )
        open_drawer.start()



    def dds_subscriber(self, domain_id,topic, message_type, task):
        domain_paticipant = DomainParticipant(domain_id=domain_id)
        topic = Topic(domain_paticipant, topic, message_type)
        subscriber = DataReader(domain_paticipant, topic)
        for message in subscriber.take_iter():
                print("step 1")
                if message.fleet_name == self.fleet_name and message.robot_name == self.name:
                    task(message)

    def ros_publish_drawer_open(self,msg):
        print("sucess")
        msg = String()
        msg.data = 'Hello World'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    ff_client = ff_fast_client()
    rclpy.spin(ff_client)   
    ff_fast_client.destroy_node()
    rclpy.shutdown()


    
if __name__ == '__main__':
    main()