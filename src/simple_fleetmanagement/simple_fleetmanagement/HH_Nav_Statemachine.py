import asyncio
from typing import Dict
from xmlrpc.client import Boolean
from .parameters_module import RobotStates, HOME_WAYPOINT_ID
from typing import Dict
from . import generic_state_machine
from asyncio import threads
from threading import Thread


class HHStateMachine:
    def __init__(self, functions_by_functionname: Dict) -> None:
        self.functions_by_functionname = functions_by_functionname
        self.check_status_function = functions_by_functionname["check_status"]
        self.current_waypoint = HOME_WAYPOINT_ID
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
            self.changeStateFunction_by_newState_by_OldState)
        self._thread_changeState = Thread(target=self.between_callback_changeState)
        self._thread_running = Thread(target=self.between_callback_running)

        # threads für die beiden funktionen initialisieren

    def start(self):
        self._thread_changeState.start()
        self._thread_running.start()

    async def some_callback_changeState(self):
        await self.state_machine.changeState()

    def between_callback_changeState(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        loop.run_until_complete(self.some_callback_changeState())
        loop.close()

    async def some_callback_running(self):
        await self.state_machine.running()

    def between_callback_running(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        loop.run_until_complete(self.some_callback_running())
        loop.close()

    def checkCondition_running_to_pause(self) -> Boolean:
        return self.check_status_function() == RobotStates.PAUSE

    def checkCondition_pause_to_drawer_open(self) -> Boolean:
        return self.check_status_function() == RobotStates.DRAWER_OPEN

    def checkCondition_pause_to_homing(self) -> Boolean:
        return self.check_status_function() == RobotStates.HOMING

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
        # loop über navgoals senden starten
        pass

    def changeState_drawer_open_to_pause(self):
        # nix. warten
        pass

    def changeState_homing_to_pause(self):
        self.functions_by_functionname["navigator_cancel_task"]()

    def stateRunning(self):
        waypoints_by_id = self.functions_by_functionname["get_waypoints_by_id"]()
        current_waypoint = self.current_waypoint
        for waypoint_id in range(current_waypoint, waypoints_by_id):
            self.current_waypoint = waypoint_id
            self.functions_by_functionname["navigate_to_pose"](waypoints_by_id[waypoint_id])
        self.current_waypoint = HOME_WAYPOINT_ID

    def statePause(self):
        # nix machen evtl sicherheitshalber hier immer cancel task für nav machen
        pass

    def stateDrawerOpen(self, drawer_id):
        if not self.functions_by_functionname["is_any_drawer_open"]():
            self.functions_by_functionname["open_drawer"](drawer_id)
        pass

    def stateHoming(self):
        self.functions_by_functionname["navigate_to_pose"](HOME_WAYPOINT_ID)
