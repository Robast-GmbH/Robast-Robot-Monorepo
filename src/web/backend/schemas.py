from typing import List, Optional, Any
from enum import Enum
import yaml
from pydantic import BaseModel
from typing import Union

class TaskTypes(str, Enum):
    Delivery = "Delivery"
    Move = "Move"
    Drawer = "Drawer"

class DrawerSlideTypes(str, Enum):
    Manual = "Manual"
    Electrical = "Electrical"

# Task
class Pose(BaseModel):
    x:  float
    y:  float
    z:  float

class Waypoint(BaseModel):
    pose: Pose
    orientation: float

class Robot(BaseModel):
    fleet_name: str
    robot_name: str

class ActionType(str, Enum):
    OPEN_DRAWER = 'OPEN_DRAWER'
    MOVE = 'MOVE'
    NEW_USER = 'NEW_USER'

class Drawer(BaseModel):
    id: int
    module_id: int
    is_edrawer: bool

class DrawerAction(Drawer):
    locked_for: list[str]

class NewUser(BaseModel):
    user_id: int

class Action(BaseModel):
    step:int
    type: ActionType
    action: Union[DrawerAction, Waypoint, NewUser]

class BaseTask(BaseModel):
    task_id:str

class Task(BaseTask):
    actions:list[Action]

class ActiveTask(Task):
    current_action : int
    robot: Robot


#Drawer
class ModuleBase(BaseModel):
    id:int
    drawer_id:int

  
class Module(ModuleBase):
    type: DrawerSlideTypes
    size: int
    robot_name: str    
    position:int
    status:str
    
    class Config:
        orm_mode = True

class UpdateModule(ModuleBase):
    robot_name: str    
    status: str

#User
class UserBase(BaseModel):
    name: str
    
class UserLogin(UserBase):
    hashed_password: str

class UserCreate(UserBase):
    hashed_password: str
    full_name: str
    email: str

class User(UserBase):
    id: int
    is_active: bool
    admin: bool
    full_name: str

    class Config:
        orm_mode = True

#Robot
class Robot(BaseModel):
    robot_name: str
    fleet_name: str  
    task_id: int
    x_pose:float
    y_pose: float
    yaw_pose: float

