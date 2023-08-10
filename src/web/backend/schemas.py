from typing import List, Optional, Any
from enum import Enum
import yaml
from pydantic import BaseModel

class TaskTypes(str, Enum):
    Delivery = "Delivery"
    Move = "Move"
    Drawer = "Drawer"

class DrawerSlideTypes(str, Enum):
    Manual = "Manual"
    Electrical = "Electrical"

#Task
class BaseAction(BaseModel):
    type: str
    position:int
    finished: bool

class Action(BaseAction):
    id: int

class MoveAction(Action):
    x_pose: float
    y_pose: float
    yaw_pose: float

class DrawerActon(Action):
    target_id :int
    drawer_id :int
    module_id :int

class CreateAction(MoveAction):
    target_id :int
    drawer_id :int
    module_id :int

class Task(BaseModel):
    id :int
    robot_name :str
    fleet_name  :str
    current_action : int
    owner_id: int
    actions:list[Action]

class CreateTask(BaseModel):
    owner_id: int
    actions:list[Action]

class UpdateTask(BaseModel):
    id: int
    current_action: int
    actions: list[Action]

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
    orders: List[Task] = []

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
