from enum import Enum


class RobotStatus(Enum):
    is_running = 1
    is_pausing = 2
    is_homing = 3


NUM_OF_DRAWERS = 5


class RobotStates(Enum):
    RUNNING = 1
    PAUSE = 2
    HOMING = 3
    DRAWER_OPEN = 4
