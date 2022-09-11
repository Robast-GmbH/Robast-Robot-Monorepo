from threading import Timer
from typing import Dict
import asyncio


class generic_state_machine(object):
    def __init__(self, stateFunction_by_state: Dict, changeStateCondition_by_newState_by_OldState, changeStateFunction_by_newState_by_OldState, initialState, runState_callback_timer=2, changeState_callback_timer=1):
        self.__stateFunction_by_state = stateFunction_by_state
        self.__changeCondition_by_AlternativeState_by_State = changeStateCondition_by_newState_by_OldState
        self.__changeStateFunction_by_newState_by_OldState = changeStateFunction_by_newState_by_OldState
        self.__current_State = initialState
        self.callback_timer = {
            "runState_callback_timer": runState_callback_timer,
            "changeState_callback_timer": changeState_callback_timer
        }

    def runState(self):
        self.__stateFunction_by_state[self.__current_State]()

    def changeState(self):
        changCondition_by_AlternativeState = self.__changeCondition_by_AlternativeState_by_State[self.__current_State]
        for alternative_state in changCondition_by_AlternativeState:
            condition_for_state_change = changCondition_by_AlternativeState[alternative_state]
            if condition_for_state_change() is True:
                print("condition change")
                self.__stateFunction_by_state[self.__current_State]()
                self.__changeStateFunction_by_newState_by_OldState[self.__current_State][alternative_state]()
                self.__current_State = alternative_state
                break
