from cyclonedds.domain import DomainParticipant
from cyclonedds.topic import Topic
from cyclonedds.sub import DataReader
from cyclonedds.util import duration
from cyclonedds.core import Listener
from cyclonedds.pub import DataWriter

from threading import Thread
import messages

class free_fleet_server:

    def __init__(self):
        self.robot_states=[] #:list[messages.FreeFleetData_RobotState] =[]
        self.start_reciving_robot_states()

    def start_reciving_robot_states(self):
        x = Thread(target=self.get_status, daemon=True )
        x.start()

    def get_status(self): 
        participant = DomainParticipant()
        topic = Topic(participant, "/robot_state",messages.FreeFleetData_RobotState )
        reader = DataReader(participant, topic)
   
        for msg in reader.take_iter():
            self.robot_states=[x for x in self.robot_states if x.name!= msg.name]  
            self.robot_states.append(msg)

    #handle Mode request
    def handle_mode_request(self, fleet_name, robot_name, mode):
        state=  self.get_robot_state(robot_name) 

        mode_request= messages.FreeFleetData_ModeRequest()
        mode_request.robot_name= robot_name
        mode_request.fleet_name= fleet_name
        mode_request.mode= mode
        mode_request.task_id= state.task_id
        mode_request.parameters= []
        self.publish_dds(mode_request, "/mode_request", messages.FreeFleetData_ModeRequest )

    #handle path request 
    def handle_path_request(self, fleet_name, robot_name, path):
        state=  self.get_robot_state(robot_name) 

        path_request= messages.FreeFleetData_PathRequest()
        path_request.robot_name= robot_name
        path_request.fleet_name= fleet_name
        path_request.task_id= state.task_id
        path_request.path= path
        self.publish_dds(path_request, "/path_request", messages.FreeFleetData_PathRequest)

    def handle_destination_request(self, fleet_name, robot_name, destination):
        state =  self.get_robot_state(robot_name) 
        
        destination_request= messages.FreeFleetData_DestinationRequest()
        destination_request.robot_name= robot_name
        destination_request.fleet_name= fleet_name
        destination_request.task_id= state.task_id
        destination_request.destination= destination
        
        self.publish_dds(destination,"/destination_request", messages.FreeFleetData_DestinationRequest)

    def publish_dds(self, message, topicPath,topicType ):
        participant = DomainParticipant()
        topic = Topic(participant, topicPath, topicType)

        listener = Listener()
        writer = DataWriter(participant, topic, listener=listener)
        writer.write( message)

    def get_robot_states(self):
        return self.robot_states 

    def get_robot_state(self, fleet_name, name):
        return [x for x in self.robot_states if x.name== name].first() 