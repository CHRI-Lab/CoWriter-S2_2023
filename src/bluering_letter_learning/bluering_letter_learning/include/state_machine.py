#!/usr/bin/env python
"""
from http://www.ibm.com/developerworks/linux/library/l-python-state/index.html
"""


class StateMachine:
    """
    Class for managing state machines.
    """

    def __init__(self):
        self.handlers = {}
        self.startState = None
        self.endStates = []
        self.currentState = None

    def add_state(self, name, handler, end_state=0):
        name = name.upper()
        self.handlers[name] = handler
        if end_state:
            self.endStates.append(name)

    def set_start(self, name):
        self.startState = name.upper()

    def get_state(self):
        return self.currentState

    def run(self, cargo):
        try:
            handler = self.handlers[self.startState]
        except:
            raise Exception(
                "InitializationError", "must call .set_start() before .run()"
            )

        if not self.endStates:
            raise Exception(
                "InitializationError", "at least one state must be an end_state"
            )

        while 1:
            (newState, cargo) = handler(cargo)
            self.currentState = newState.upper()
            if self.currentState in self.endStates:
                break
            else:
                handler = self.handlers[self.currentState]
