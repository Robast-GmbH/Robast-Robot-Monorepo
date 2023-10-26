from typing import Optional
from enum import Enum
from pydantic import BaseModel
from typing import Union


class DrawerSlideTypes(str, Enum):
    MANUAL = "Manual"
    ELECTRICAL = "Electrical"


# Robot
class Robot(BaseModel):
    fleet_name: str
    robot_name: str


class RobotStatus(Robot):
    task_id: Optional[int]
    x_pose: float
    y_pose: float
    yaw_pose: float
    battery_level: float


# Task
class Pose(BaseModel):
    x:  float
    y:  float
    z:  float


class Navigation(BaseModel):
    pose: Pose
    yaw: float


class ActionType(str, Enum):
    DRAWER = 'DRAWER'
    NAVIGATION = 'NAVIGATION'
    NEW_USER = 'NEW_USER'


class BaseDrawer(BaseModel):
    drawer_id: int
    module_id: int


class Drawer(BaseDrawer):
    locked_for: list[int]


class NewUser(BaseModel):
    user_id: int


class UpdateAction(BaseModel):
    status: str
    finished: bool


class Action(BaseModel):
    step: int
    type: ActionType
    action: Union[Drawer, Navigation, NewUser]
    finished: bool


class BaseTask(BaseModel):
    task_id: str
    robot: Robot


class Task(BaseTask):
    actions: list[Action]


# Drawer
class ModuleBase(BaseModel):
    module_id: int
    drawer_id: int


class Module(ModuleBase):
    type: DrawerSlideTypes
    size: int
    robot_name: str
    position: int
    status: str
    label: str

    class Config:
        orm_mode = True


class UpdateModule(ModuleBase):
    robot_name: Union[str, None] = None
    status: Union[str, None] = None
    label: Union[str, None] = None


# User
class UserBase(BaseModel):
    name: str


class UserLogin(UserBase):
    hashed_password: str


class UserCreate(UserBase):
    hashed_password: str
    full_name: str
    email: str
    admin: bool


class User(UserBase):
    id: Union[int, None] = None
    is_active: bool = True
    admin: bool
    full_name: str

    class Config:
        orm_mode = True
