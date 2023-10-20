#!/usr/bin/env python3
"""
This is an example of a state machine implemented in Python 3, adapted from:
http://www.ibm.com/developerworks/linux/library/l-python-state/index.html
"""

from typing import Callable, Dict, List, Optional

# Use .upper() method of str; string.upper import is not available in Python 3
# from string import upper


class StateMachine:
    """
    Class for managing state machines.

    Attributes
    ----------
    handlers : Dict
        A dictionary containing handler functions for each state.
    start_state : str
        The starting state of the state machine.
    end_states : List
        A list of end states for the state machine.
    current_state : str
        The current state of the state machine.
    """

    def __init__(self):
        self.handlers: Dict = {}
        self.start_state: Optional[str] = None
        self.end_states: List = []
        self.current_state: Optional[str] = None

    def add_state(
        self, name: str, handler: Optional[Callable], end_state: Optional[bool] = None
    ) -> None:
        """
        Add a new state to the state machine, along with its
        corresponding handler function.

        Parameters
        ----------
        name : str
            The name of the state to be added.
        handler : callable
            The function to be called when the state is active.
        end_state : bool, optional
            A flag indicating if the state is an end state, by default
            False.
        """
        if end_state is None:
            end_state = False

        name = name.upper()
        self.handlers[name] = handler
        if end_state:
            self.end_states.append(name)

    def set_start(self, name: str) -> None:
        """
        Set the starting state of the state machine.

        Parameters
        ----------
        name : str
            The name of the starting state.
        """
        self.start_state = name.upper()
        self.current_state = self.start_state

    def get_state(self) -> Optional[str]:
        """
        Return the current state of the state machine.
        """
        return self.current_state

    def run(self, cargo) -> None:
        """
        Run the state machine starting from the initial state,
        processing input (cargo) until an end state is reached.

        Parameters
        ----------
        cargo
            The input data to be processed by the state machine.
        """
        try:
            handler = self.handlers[self.start_state]
        # Updated exception raising to use Python 3 syntax
        except KeyError:
            raise Exception("InitializationError: must call .set_start() before .run()")

        if not self.end_states:
            raise Exception(
                "InitializationError: at least one state must be an end_state"
            )

        while True:
            new_state, cargo = handler(cargo)
            self.current_state = new_state.upper()
            if self.current_state in self.end_states:
                break
            else:
                handler = self.handlers[self.current_state]
