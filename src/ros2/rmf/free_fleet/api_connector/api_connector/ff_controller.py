from cyclonedds.domain import DomainParticipant
from cyclonedds.topic import Topic
from cyclonedds.sub import DataReader
from cyclonedds.core import Listener
from cyclonedds.pub import DataWriter
from . import messages

from threading import Thread


class free_fleet_controller:

    def __init__(self):
        # self.declare_parameter('dds_domain', 42)
        # self.declare_parameter('dds_robot_state_topic', "/robot_state")
        # self.declare_parameter('dds_mode_request_topic', "/mode_request")
        # self.declare_parameter('dds_path_request_topic', "/path_request")
        # self.declare_parameter('dds_destination_request_topic', "/destination_request")
        # self.declare_parameter('dds_open_drawer_topic', "/OpenDrawerequest")

        # self.dds_domain = self.get_parameter('dds_domain').get_parameter_value().integer_value
        # self.dds_robot_state_topic = self.get_parameter('dds_robot_state_topic').get_parameter_value().string_value
        # self.dds_mode_request_topic = self.get_parameter('dds_mode_request_topic').get_parameter_value().string_value
        # self.dds_dds_path_request_topic = self.get_parameter('dds_path_request_topic').get_parameter_value().string_value
        # self.dds_destination_request_topic = self.get_parameter('dds_destination_request_topic').get_parameter_value().string_value
        # self.dds_open_drawer_topic = self.get_parameter('dds_open_drawer_topic').get_parameter_value().string_value

        self.dds_domain = 42
        self.dds_robot_state_topic = "/robot_state"
        self.dds_mode_request_topic = "/mode_request"
        self.dds_dds_path_request_topic = "/path_request"
        self.dds_destination_request_topic = "/destination_request"
        self.dds_open_drawer_topic = "/OpenDraweRequest"

        self.robot_states = []  # :list[messages.FreeFleetData_RobotState] =[]
        self.start_reciving_robot_states()

    def start_reciving_robot_states(self):
        x = Thread(target=self.get_status, daemon=True)
        x.start()

    def get_status(self):
        self.participant = DomainParticipant(domain_id=self.dds_domain)
        topic = Topic(
            self.participant,
            self.dds_robot_state_topic,
            messages.FreeFleetData_RobotState)

        self.reader = DataReader(self.participant, topic)

        for msg in self.reader.take_iter():
            self.robot_states = [x for x in self.robot_states if x.name != msg.name]
            self.robot_states.append(msg)

    def get_robot_status(self, robot_name):
        return list(filter(lambda robot: robot.name == robot_name, self.robot_states))

    # handle Mode request
    def handle_mode_request(self, fleet_name, robot_name, mode):
        state = self.get_robot_state(robot_name)
        mode_request = messages.FreeFleetData_ModeRequest()
        mode_request.robot_name = robot_name
        mode_request.fleet_name = fleet_name
        mode_request.mode = mode
        mode_request.task_id = state.task_id
        mode_request.parameters = []
        self.publish_dds(
            mode_request,
            self.dds_mode_request_topic,
            messages.FreeFleetData_ModeRequest)

    # handle path request
    def handle_path_request(self, fleet_name, robot_name, path):
        state = self.get_robot_state(robot_name)
        path_request = messages.FreeFleetData_PathRequest()
        path_request.robot_name = robot_name
        path_request.fleet_name = fleet_name
        path_request.task_id = state.task_id
        path_request.path = path
        self.publish_dds(
            path_request,
            self.dds_dds_path_request_topic,
            messages.FreeFleetData_PathRequest)

    # handle destination request
    def handle_destination_request(self, fleet_name, robot_name, destination):
        state = self.get_robot_state(fleet_name, robot_name)
        destination_request = messages.FreeFleetData_DestinationRequest()
        destination_request.robot_name = robot_name
        destination_request.fleet_name = fleet_name
        destination_request.task_id = state.task_id
        destination_request.destination = destination
        self.publish_dds(
            destination_request,
            self.dds_destination_request_topic,
            messages.FreeFleetData_DestinationRequest)

    # handle_open_drawer_request
    def handle_open_drawer_request(self, fleet_name, robot_name, module_id, drawer_id):
        open_drawer_request = messages.FreeFleetData_OpenDrawerRequest(
            fleet_name,
            robot_name,
            module_id,
            drawer_id)

        self.publish_dds(
            open_drawer_request,
            self.dds_open_drawer_topic,
            messages.FreeFleetData_OpenDrawerRequest)

    def publish_dds(self, message, topicPath, topicType):
        topic = Topic(self.participant, topicPath, topicType)
        listener = Listener()
        writer = DataWriter(self.participant, topic, listener=listener)
        writer.write(message)

    def get_robot_states(self):
        return self.robot_states

    def get_robot_state(self, fleet_name, name):
        return [x for x in self.robot_states if x.name == name].first()
