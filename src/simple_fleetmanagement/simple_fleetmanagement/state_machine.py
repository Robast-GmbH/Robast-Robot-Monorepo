from typing import Dict


class state_machine(object):
    def __init__(self, stateFunction_by_state: Dict, change_condition_by_state):
        self.stateFunction_by_state = stateFunction_by_state
        self.change_condition_by_state = change_condition_by_state
        self.stop = False
        self.current_State = None

    def setNumberOfPositions(self, count: int):
        self.number_of_waypoints = count
        self.current_waypoint = 0  # als reset, wenn die anzahl der waypoints geändert wird

    def statemachineMovement(self):
        for i in range(self.number_of_waypoints):
            self.go_to_pos_function(i)
            self.current_waypoint = i
        print("finished all waypoints")

    def running(self):
        self.current_State = next(iter(self.stateFunction_by_state))
        while(not self.stop and self.current_State != None):
            pass

    def stateRunning(self):
        pass

    def statePause(self):
        pass

    def stateHoming(self):
        pass

    def stateDrawerOpen(self):
        pass
