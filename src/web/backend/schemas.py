from typing import List, Optional, Any
from enum import Enum
import yaml

from pydantic import BaseModel


class TaskTypes(str, Enum):
    Delivery = "Delivery"
    Move = "Move"
    Drawer = "Drawer"

class DrawerTypes(str, Enum):
    Manual = "Manual"
    Electrical = "Electrical"



#Task
class TaskBase(BaseModel):
   pass
   
class TaskDelivery(TaskBase):
   
    owner_id: int
    target_id: int
    module_id: int
    drawer_id: int
    x_pose:float
    y_pose: float
    yaw_pose: float

class TaskDrawer(TaskBase):
   
    owner_id: int
    target_id: int
    module_id: int
    drawer_id: int


class TaskMove(TaskBase):
    x_pose:float
    y_pose: float
    yaw_pose: float

class TaskUpdate(BaseModel):
    id: int 
    fleet: str
    robot: str
    status:str
    finished: bool

class Task(TaskDelivery):
    id: int 
    fleet: str
    robot: str
    status:str
    finished: bool

#Drawer
class ModuleBase(BaseModel):
    id:int
    drawer_id:int
    type: DrawerTypes
    fleet_name: str
    robot_name: str

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
