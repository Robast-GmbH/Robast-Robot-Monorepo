import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.node import Node
from std_msgs.msg import String
from cyclonedds.domain import DomainParticipant
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from . import messages
from threading import Thread
from communication_interfaces.msg import DrawerAddress

class free_fleet_client_direct(Node):

    def __init__(self):
        super().__init__('free_fleet_direct_client')
        
        self.declare_parameter('fleet_name', 'ROBAST_1')
        self.declare_parameter('robot_name', 'RB0')
        self.declare_parameter('statemaschine_open_drawer_topic','/trigger_drawer_tree')
        
        self.declare_parameter('dds_domain', 42)
        self.declare_parameter('dds_open_drawer_topic', '/OpenDrawerRequest')
        
        self.fleet_name = self.get_parameter('fleet_name').get_parameter_value().string_value
        self.name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.ros_opendrawer_topic = self.get_parameter('statemaschine_open_drawer_topic').get_parameter_value().string_value

        self.dds_domain = self.get_parameter('dds_domain').get_parameter_value().integer_value
        self.dds_open_drawer_topic=self.get_parameter('dds_open_drawer_topic').get_parameter_value().string_value

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability= QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.publisher_ = self.create_publisher(DrawerAddress, self.ros_opendrawer_topic, qos_profile=qos_profile)
        self.start_receiving_from_dds()
       
    def start_receiving_from_dds(self):
        open_drawer = Thread(target=self.dds_subscriber, args=(self.dds_domain, self.dds_open_drawer_topic, messages.FreeFleetData_OpenDrawerRequest, self.ros_publish_drawer_open ), daemon=True )
        open_drawer.start()

    def dds_subscriber(self, domain_id,topic_name, message_type, task):
        domain_participant = DomainParticipant(domain_id=domain_id)
        topic = Topic(domain_participant, topic_name, message_type)
        subscriber = DataReader(domain_participant, topic)
        print("doimain id "+ str(domain_id))
        print( topic_name)

        for message in subscriber.take_iter():
                print("message recived")
                if message.fleet_name == self.fleet_name and message.robot_name == self.name:
                     print("Message ok")
                     task(message) 
                   

    def ros_publish_drawer_open(self,dds_msg):
        print("ros pub")
        ros_msg = DrawerAddress()
        ros_msg.module_id = dds_msg.module_id
        ros_msg.drawer_id = dds_msg.drawer_id
       
        self.publisher_.publish(ros_msg)
  

def main(args=None):
    rclpy.init(args=args)
    ff_client = free_fleet_client_direct()
    rclpy.spin(ff_client)   
    free_fleet_client_direct.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
