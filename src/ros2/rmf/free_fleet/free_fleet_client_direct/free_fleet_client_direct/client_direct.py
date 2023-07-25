import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.node import Node
from cyclonedds.domain import DomainParticipant
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from tf.transformations import quaternion_from_euler
from . import messages
from threading import Thread
from communication_interfaces.msg import DrawerAddress
from geometry_msgs import PoseStamped


class free_fleet_client_direct(Node):

    def __init__(self):
        super().__init__('free_fleet_direct_client')

        self.declare_parameter('fleet_name', 'ROBAST_1')
        self.declare_parameter('robot_name', 'RB0')
        self.declare_parameter('statemaschine_open_drawer_topic', 'trigger_drawer_tree')
        self.declare_parameter('statemaschine_open_e_drawer_topic', 'trigger_electric_drawer_tree')
        self.declare_parameter('move_base_server_name', 'goal_pose ')

        self.declare_parameter('dds_domain', 42)
        self.declare_parameter('dds_open_drawer_topic', '/OpenDrawerRequest')
        self.declare_parameter('dds_nav_goal_topic', '/MoveToRequest')

        self.fleet_name = self.get_parameter('fleet_name').get_parameter_value().string_value
        self.name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.ros_opendrawer_topic = self.get_parameter('statemaschine_open_drawer_topic')
        .get_parameter_value().string_value
        self.ros_open_e_drawer_topic = self.get_parameter('statemaschine_open_e_drawer_topic')
        .get_parameter_value().string_value
        self.ros_move_base_server_name=self.get_parameter('move_base_server_name')
        .get_parameter_value().string_value
        self.dds_domain = self.get_parameter('dds_domain').get_parameter_value().integer_value
        self.dds_open_drawer_topic=self.get_parameter('dds_open_drawer_topic').get_parameter_value().string_value
        self.dds_move_to_topic=self.get_parameter('dds_nav_goal_topic').get_parameter_value().string_value 

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        # ToDo Torben:fix for real multirobot support by placing them into there namespaces
        self.drawer_publisher_ = self.create_publisher(DrawerAddress, self.ros_opendrawer_topic, qos_profile=qos_profile)
        self.e_drawer_publisher_ = self.create_publisher(DrawerAddress, self.ros_open_e_drawer_topic, qos_profile=qos_profile)
        self.move_publisher_ = self.create_publisher(PoseStamped, self.ros_move_base_server_name)
        self.start_receiving_from_dds()

    def start_receiving_from_dds(self):
        open_drawer = Thread(target=self.dds_subscriber, args=(self.dds_domain, self.dds_open_drawer_topic, messages.FreeFleetData_OpenDrawerRequest, self.ros_publish_drawer_open), daemon=True)
        open_drawer.start()
        move_to = Thread(target=self.dds_subscriber, args=(self.dds_domain, self.dds_move_to_topic, messages.FreeFleetData_DestinationRequest, self.ros_publish_robot_move_to), daemon=True)
        move_to.start()

    def dds_subscriber(self, domain_id, topic_name, message_type, task):
        domain_participant = DomainParticipant(domain_id=domain_id)
        topic = Topic(domain_participant, topic_name, message_type)
        subscriber = DataReader(domain_participant, topic)
        for message in subscriber.take_iter():
            if message.fleet_name == self.fleet_name and message.robot_name == self.name:
                task(message)

    def ros_publish_drawer_open(self, dds_msg):
        ros_msg = DrawerAddress()
        ros_msg.module_id = dds_msg.module_id
        ros_msg.drawer_id = dds_msg.drawer_id

        if(dds_msg.e_drawer):
            self.e_drawer_publisher_(ros_msg)
        else:
            self.drawer_publisher_(ros_msg)

    def ros_publish_robot_move_to(self, dds_msg):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.__node.get_clock().now()
        goal_msg.pose.position.x = dds_msg.destination.X
        goal_msg.pose.position.y = dds_msg.destination.Y
        goal_msg.pose.orientation = quaternion_from_euler(0, 0, dds_msg.destination.yaw)
        goal_msg.goalMsg.header.frame_id = dds_msg.robot_name+"/map"
        self.pub.publish(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    ff_client = free_fleet_client_direct()
    rclpy.spin(ff_client)
    free_fleet_client_direct.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
