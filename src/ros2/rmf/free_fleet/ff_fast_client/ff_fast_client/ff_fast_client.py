import rclpy
from std_msgs.msg import String
from rclpy.node import Node
from cyclonedds.domain import DomainParticipant
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from . import messages
from threading import Thread
from communication_interfaces.msg import DrawerAddress

class ff_fast_client(Node):

    def __init__(self):
        super().__init__('ff_fast_client')
        
        self.declare_parameter('fleet_name', 'ROBAST_1')
        self.declare_parameter('robot_name', 'RB0')
        self.declare_parameter('statemaschine_opendrawer_topic','trigger_drawer_tree')
        
        self.declare_parameter('dds_domain', 42)
        self.declare_parameter('dds_opendrawer_topic', 'OpenDrawerequest')
        
        self.fleet_name = self.get_parameter('fleet_name').get_parameter_value().string_value
        self.name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.ros_opendrawer_topic = self.get_parameter('statemaschine_opendrawer_topic').get_parameter_value().string_value

        self.dds_domain = self.get_parameter('dds_domain').get_parameter_value().integer_value
        self.dds_open_drawer_topic=self.get_parameter('dds_opendrawer_topic').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(DrawerAddress, 'dds_opendrawer_topic', 10)
        self.start_receiving_from_dds()
       
    def start_receiving_from_dds(self):
        open_drawer = Thread(target=self.dds_subscriber, args=(self.dds_domain, self.dds_opendrawer_topic, self.dds_open_drawer_topic,messages.FreeFleetData_OpenDrawerRequest,self.ros_publish_drawer_open ), daemon=True )
        open_drawer.start()

    def dds_subscriber(self, domain_id,topic, message_type, task):
        domain_paticipant = DomainParticipant(domain_id=domain_id)
        topic = Topic(domain_paticipant, topic, message_type)
        subscriber = DataReader(domain_paticipant, topic)
        for message in subscriber.take_iter():
               
                if message.fleet_name == self.fleet_name and message.robot_name == self.name:
                    task(message) 

    def ros_publish_drawer_open(self,dds_msg):
        ros_msg = DrawerAddress()
        ros_msg.module_id = dds_msg.module_id
        ros_msg.drawer_id = dds_msg.drawer_id
        self.publisher_.publish(ros_msg)
  
def main(args=None):
    rclpy.init(args=args)
    ff_client = ff_fast_client()
    rclpy.spin(ff_client)   
    ff_fast_client.destroy_node()
    rclpy.shutdown()


    
if __name__ == '__main__':
    main()