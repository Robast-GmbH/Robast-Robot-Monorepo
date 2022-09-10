from typing import Dict
import asyncio


class generic_state_machine(object):
    def __init__(self, stateFunction_by_state: Dict, changeStateCondition_by_newState_by_OldState, changeStateFunction_by_newState_by_OldState):
        self.__stateFunction_by_state = stateFunction_by_state
        self.__changeCondition_by_AlternativeState_by_State = changeStateCondition_by_newState_by_OldState
        self.__changeStateFunction_by_newState_by_OldState = changeStateFunction_by_newState_by_OldState
        self.stop_state_machine = False
        self.__current_State = None
        self.__sem = asyncio.Semaphore(1)

    async def running(self):
        self.__current_State = next(iter(self.__stateFunction_by_state))
        while(not self.stop_state_machine and self.__current_State != None):
            self.__stateFunction_by_state[self.__current_State]()
        return True

    async def changeState(self):
        while(not self.stop_state_machine and self.__current_State != None):
            changCondition_by_AlternativeState = self.__changeCondition_by_AlternativeState_by_State[self.__current_State]
            for alternative_state in changCondition_by_AlternativeState:
                condition_for_state_change = changCondition_by_AlternativeState[alternative_state]
                if condition_for_state_change() is True:
                    self.__stateFunction_by_state[self.__current_State]()
                    self.__changeStateFunction_by_newState_by_OldState[self.__current_State][alternative_state]()
                    self.__current_State = alternative_state
                    break
        return True
