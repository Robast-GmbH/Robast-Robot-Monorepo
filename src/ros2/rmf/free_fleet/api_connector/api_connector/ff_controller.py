
from . import messages

from threading import Thread


class free_fleet_controller:

    def __init__(self, dds_config: dict):
        self.config = dds_config
        self.robot_states = []  # :list[messages.FreeFleetData_RobotState] =[]
        self.start_reciving_robot_states()

    def start_reciving_robot_states(self):
        x = Thread(target=self.get_status, daemon=True)
        x.start()

    def get_status(self):
        participant = DomainParticipant(domain_id=self.config["dds_domain"])
        topic = Topic(
            participant,
            self.config["robot_state_topic"],
            messages.FreeFleetData_RobotState)

        self.reader = DataReader(participant, topic)

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
            self.config["mode_request_topic"],
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
            self.config["path_request_topic"],
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
            self.config["destination_request_topic"],
            messages.FreeFleetData_DestinationRequest)

    # handle_open_drawer_request
    def handle_slide_drawer_request(self, fleet_name:str, robot_name: str, module_id: int, drawer_id: int , e_drawer: bool, open: bool):
        slide_drawer_request = messages.FreeFleetData_SlideDrawerRequest(
            fleet_name,
            robot_name,
            module_id,
            drawer_id,
            e_drawer,
            open)

        self.publish_dds(
            slide_drawer_request,
            self.config["slide_drawer_topic"],
            messages.FreeFleetData_SlideDrawerRequest)
        

    

    def get_robot_states(self):
        return self.robot_states

    def get_robot_state(self, fleet_name, name):
        return [x for x in self.robot_states if x.name == name].first()
