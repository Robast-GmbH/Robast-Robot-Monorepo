import asyncio
from typing import Dict
from xmlrpc.client import Boolean
from .parameters_module import RobotStates, HOME_WAYPOINT_ID
from typing import Dict
from . import generic_state_machine
from time import sleep


class HHStateMachine:
    def __init__(self, functions_by_functionname: Dict) -> None:
        self.functions_by_functionname = functions_by_functionname
        self.check_status_function = functions_by_functionname["check_status"]
        self.current_waypoint = HOME_WAYPOINT_ID
        self.active = False
        self.stateFunction_by_state = {
            RobotStates.PAUSE: self.statePause,
            RobotStates.RUNNING: self.stateRunning,
            RobotStates.HOMING: self.stateHoming,
            RobotStates.DRAWER_OPEN: self.stateDrawerOpen,
        }
        self.changeStateFunction_by_newState_by_OldState = {
            RobotStates.PAUSE: {
                RobotStates.DRAWER_OPEN: self.changeState_pause_to_drawer_open,
                RobotStates.HOMING: self.changeState_pause_to_homing,
                RobotStates.RUNNING: self.changeState_pause_to_running
            },
            RobotStates.RUNNING: {
                RobotStates.PAUSE: self.changeState_running_to_pause
            },
            RobotStates.HOMING: {RobotStates.PAUSE: self.changeState_homing_to_pause},
            RobotStates.DRAWER_OPEN: {RobotStates.PAUSE: self.changeState_drawer_open_to_pause}
        }
        self.changeStateCondition_by_newState_by_OldState = {
            RobotStates.PAUSE: {
                RobotStates.DRAWER_OPEN: self.checkCondition_pause_to_drawer_open,
                RobotStates.HOMING: self.checkCondition_pause_to_homing,
                RobotStates.RUNNING: self.checkCondition_pause_to_running
            },
            RobotStates.RUNNING: {
                RobotStates.PAUSE: self.checkCondition_running_to_pause
            },
            RobotStates.HOMING: {RobotStates.PAUSE: self.checkCondition_homing_to_pause},
            RobotStates.DRAWER_OPEN: {RobotStates.PAUSE: self.checkCondition_drawer_open_to_pause}
        }

        self.state_machine = generic_state_machine.generic_state_machine(
            self.stateFunction_by_state,
            self.changeStateCondition_by_newState_by_OldState,
            self.changeStateFunction_by_newState_by_OldState,
            self.check_status_function())

        # threads für die beiden funktionen initialisieren

    def run(self):
        self.state_machine.changeState()
        self.state_machine.runState()

    def check_state(self):
        self.state_machine.changeState()

    def run_state(self):
        self.state_machine.runState()

    def checkCondition_running_to_pause(self) -> Boolean:
        return self.check_status_function() == RobotStates.PAUSE

    def checkCondition_pause_to_drawer_open(self) -> Boolean:
        return self.check_status_function() == RobotStates.DRAWER_OPEN

    def checkCondition_pause_to_homing(self) -> Boolean:
        return self.check_status_function() == RobotStates.HOMING

    def checkCondition_pause_to_specific_move(self) -> Boolean:
        return self.check_status_function() == RobotStates.SPECIAL

    def checkCondition_pause_to_running(self) -> Boolean:
        return self.check_status_function() == RobotStates.RUNNING

    def checkCondition_drawer_open_to_pause(self) -> Boolean:
        if (self.check_status_function() == RobotStates.PAUSE) and self.functions_by_functionname["is_any_drawer_open"]():
            return True
        else:
            return False

    def checkCondition_homing_to_pause(self) -> Boolean:
        return self.check_status_function() == RobotStates.PAUSE

    def changeState_running_to_pause(self):
        self.functions_by_functionname["navigator_cancel_task"]()

    def changeState_pause_to_drawer_open(self):
        pass

    def changeState_pause_to_homing(self):
        pass

    def changeState_pause_to_running(self):
        pass

    def changeState_pause_to_specific_move(self):

        pass

    def changeState_drawer_open_to_pause(self):
        pass

    def changeState_specific_move_to_pause(self):
        self.active = False

    def changeState_homing_to_pause(self):
        self.functions_by_functionname["navigator_cancel_task"]()
        self.active = False

    def stateRunning(self):
        if self.active == True or self.check_status_function() != RobotStates.RUNNING:
            return
        elif(self.active == False and self.functions_by_functionname["is_navigator_Task_complete"]()):
            self.active = True
            self.functions_by_functionname["navigate_to_pose"](self.current_waypoint)
            if((self.current_waypoint) < len(self.functions_by_functionname["get_waypoints_by_id"]())):
                print("waypoint erreicht")
                self.current_waypoint += 1
                sleep(30)
            else:
                self.current_waypoint = 1
            self.active = False

    def statePause(self):
        # nix machen evtl sicherheitshalber hier immer cancel task für nav machen
        self.functions_by_functionname["navigator_cancel_task"]()

    def stateDrawerOpen(self, drawer_id):
        if not self.functions_by_functionname["is_any_drawer_open"]():
            self.functions_by_functionname["open_drawer"](drawer_id)

    def stateHoming(self):
        if(self.active == False and self.functions_by_functionname["is_navigator_Task_complete"]()):
            self.active = True
            self.functions_by_functionname["navigate_to_pose"](HOME_WAYPOINT_ID)
            self.active = False
            self.functions_by_functionname["set_state_in_backend"](RobotStates.PAUSE)

    def moveToSpecificWaypoint(self):
        if(self.active == False and self.functions_by_functionname["is_navigator_Task_complete"]()):
            self.active = True
            self.functions_by_functionname["navigate_to_pose"](self.functions_by_functionname["get_goal"])
            self.active = False
            self.functions_by_functionname["set_state_in_backend"](RobotStates.PAUSE)
