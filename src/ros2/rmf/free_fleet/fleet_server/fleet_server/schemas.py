from enum import Enum
from pydantic import BaseModel
from typing import Union


class Pose(BaseModel):
    x:  float
    y:  float
    z:  float


class Waypoint(BaseModel):
    pose: Pose
    yaw: float


class Robot(BaseModel):
    fleet_name: str
    robot_name: str


class Drawer(BaseModel):
    drawer_id: int
    module_id: int
    is_edrawer: bool


class DrawerAction(Drawer):
    locked_for: list[int]


class NewUser(BaseModel):
    user_id: int


class ActionType(str, Enum):
    OPEN_DRAWER = 'OPEN_DRAWER'
    NAVIGATION = 'NAVIGATION'
    NEW_USER = 'NEW_USER'


class Action(BaseModel):
    phase: int
    type: ActionType
    action: Union[DrawerAction, Waypoint, NewUser]


class Task(BaseModel):
    robot: Robot
    task_id: str
    actions: list[Action]
