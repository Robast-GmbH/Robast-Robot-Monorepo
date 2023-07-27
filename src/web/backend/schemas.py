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
   

class TaskCreate(TaskBase):
    robot: str
    fleet: str
    task_type: str    
    owner_id: int
    drawer_id: int
    x_pose:float
    y_pose: float
    yaw_pose: float


class TaskUpdate(TaskBase):
    status:str
    finished: bool


class Task(TaskCreate):
    id: int
    status:str
    finished: bool


#Drawer
class ModuleBase(BaseModel):
    id:int
    drawer_id:int
    type:str
    
class ModuleCreate(ModuleBase):
    pass

class Module(ModuleBase):
    pass

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

#
class Robot(BaseModel):
    robot_name: str
    fleet_name: str  
    task_id: int
    x_pose:float
    y_pose: float
    yaw_pose: float
  