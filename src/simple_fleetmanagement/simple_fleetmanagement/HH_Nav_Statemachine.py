from xmlrpc.client import Boolean
from parameters_module import RobotStates


class HHStateMachine:
    def __init__(self, checkStatusFunc) -> None:
        self.check_Status_function = checkStatusFunc
        pass

    def start(self):
        pass

    def checkCondition_running_to_pause(self) -> Boolean:
        return self.check_Status_function() == RobotStates.PAUSE

    def checkContition_pause_to_drawer_open(self) -> Boolean:
        return self.check_Status_function() == RobotStates.DRAWER_OPEN

    def checkCondition_pause_to_homing(self) -> Boolean:
        return self.check_Status_function() == RobotStates.HOMING

    def checkCondition_pause_to_running(self) -> Boolean:
        return self.check_Status_function() == RobotStates.RUNNING

    def checkCondition_drawer_open_to_pause(self) -> Boolean:
        if (self.check_Status_function() == RobotStates.PAUSE) and True:  # TODO true ersetzen mit topic frage nach alle zu
            return True
        else:
            return False

    def checkCondition_homing_to_pause(self) -> Boolean:
        return self.check_Status_function() == RobotStates.PAUSE

    def changeState_running_to_pause(self):
        # nav pause/stop senden
        pass

    def changeState_pause_to_drawer_open(self):
        # öffne passende schublade
        pass

    def changeState_pause_to_homing(self):
        # send navgoal zu waipoint 0
        pass

    def changeState_pause_to_running(self):
        # loop über navgoals senden starten
        pass

    def changeState_drawer_open_to_pause(self):
        # nix. warten
        pass

    def stateRunning(self):
        # while über alle navgoals, mit warten an jedem goal
        pass

    def statePause(self):
        # nix machen
        pass

    def stateDrawerOpen(self):
        # wait/nix machen
        pass

    def stateHoming(self):
        # nav to waypoint 0
        pass
