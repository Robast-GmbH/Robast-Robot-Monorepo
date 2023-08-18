from typing import List, Optional, Any
from enum import Enum
import yaml
from pydantic import BaseModel
from typing import Union

class DrawerSlideTypes(str, Enum):
    Manual = "Manual"
    Electrical = "Electrical"

# Task
class Pose(BaseModel):
    x:  float
    y:  float
    z:  float

class Navigation(BaseModel):
    pose: Pose
    orientation: float

class Robot(BaseModel):
    fleet_name: str
    robot_name: str

class ActionType(str, Enum):
    DRAWER = 'DRAWER'
    NAVIGATION = 'NAVIGATION'
    NEW_USER = 'NEW_USER'

class BaseDrawer(BaseModel):
    drawer_id: int
    module_id: int


class Drawer(BaseDrawer):
    locked_for: list[str]

class NewUser(BaseModel):
    user_id: int

class UpdateAction(BaseModel):
    status:str
    finished:bool

class Action(BaseModel):
    step:int
    type: ActionType
    action: Union[Drawer, Navigation, NewUser]

class BaseTask(BaseModel):
    task_id:str

class UpdateTask(BaseModel):
    robot: Robot
    
class Task(BaseTask):
    actions:list[Action]


#Drawer
class ModuleBase(BaseModel):
    module_id:int
    drawer_id:int

  
class Module(ModuleBase):
    type: DrawerSlideTypes
    size: int
    robot_name: str    
    position:int
    status:str
    label:str
    
    class Config:
        orm_mode = True

class UpdateModule(ModuleBase):
    robot_name: str    
    status: str
    label:str

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
class RobotStatus(BaseModel):
    robot_name: str
    fleet_name: str  
    task_id: str
    x_pose:float
    y_pose: float
    yaw_pose: float
    battery_level:float

