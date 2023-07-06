
from dataclasses import dataclass
from cyclonedds.idl import IdlStruct
import cyclonedds.idl as idl
from cyclonedds.idl.types import   uint32, sequence

# should be moved in its own project

@dataclass
class FreeFleetData_RobotMode(IdlStruct):
    mode: uint32  
    
@dataclass
class FreeFleetData_Location(IdlStruct):
    sec: uint32
    nanosec: uint32
    x: float
    y: float
    yaw: float
    level_name: str 

@dataclass
class FreeFleetData_RobotState(IdlStruct):
    name: str 
    model:str
    task_id: str
    mode: FreeFleetData_RobotMode 
    battery_percent: float 
    location: FreeFleetData_Location 
    path: sequence[FreeFleetData_Location] 

@dataclass
class FreeFleetData_ModeParameter(IdlStruct):
    name: str 
    value:str
 
@dataclass
class  FreeFleetData_ModeRequest(IdlStruct):
    fleet_name: str
    robot_name: str
    mode: FreeFleetData_RobotMode 
    task_id: str
    parameters:  sequence[FreeFleetData_ModeParameter]

@dataclass
class  FreeFleetData_PathRequest(IdlStruct):
    fleet_name: str 
    robot_name: str
    path:  sequence[FreeFleetData_Location]
    task_id: str

@dataclass
class  FreeFleetData_DestinationRequest(IdlStruct):
    fleet_name: str
    robot_name: str
    destination: FreeFleetData_Location
    task_id: str

@dataclass
class  FreeFleetData_OpenDrawerRequest(IdlStruct):
    fleet_name: str
    robot_name: str
    module_id: int
    drawer_id: int
 