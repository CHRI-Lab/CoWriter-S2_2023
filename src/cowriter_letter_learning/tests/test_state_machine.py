#!/usr/bin/env python3
"""
Test module for StateMachine.

This module provides test functions and a test class to test the StateMachine class.
"""
import os
import rospy
import rostest
import sys
import unittest
from typing import Any, Tuple
sys.path.insert(
    0, 
    os.path.join(os.path.dirname(os.path.abspath(__file__)), '../include'))
from state_machine import StateMachine


def state_a_handler(cargo: Any) -> Tuple[str, Any]:
    """
    Handler function for state A.

    Transitions to state B if cargo is "go_to_b", otherwise remains in 
    state A.


    Parameters
    ----------
    cargo : Any
        The input data for the state.

    Returns
    -------
    tuple
        The new state and the updated cargo.
    """
    if cargo == "go_to_b":
        return "STATE_B", "finished"
    return "STATE_A", "go_to_b"


class TestStateMachine(unittest.TestCase):
    """
    Test class for the StateMachine class.
    """

    def test_state_machine(self) -> None:
        """
        Test the state machine transitions and error handling.

        This test case creates a StateMachine instance, adds states and
        their handlers, sets the starting state, and then tests the 
        transitions and error handling by running the state machine with
        different cargo inputs.
        """
        # Create a StateMachine instance
        state_machine = StateMachine()
        # Add states and their handlers to the state machine
        state_machine.add_state("STATE_A", state_a_handler)
        state_machine.add_state("STATE_B", lambda cargo: (
            "STATE_B", cargo), end_state=True)
        # Set the start state
        state_machine.set_start("STATE_A")

        # Run the state machine and test transitions
        state_machine.run("go_to_b")
        self.assertEqual(state_machine.get_state(), "STATE_B")

        # Create another StateMachine instance without an end state
        state_machine2 = StateMachine()
        state_machine2.add_state("STATE_A", state_a_handler)
        state_machine2.set_start("STATE_A")

        # Test exceptions when running state machine without end state
        with self.assertRaises(Exception) as context:
            state_machine2.run("go_to_b")
        self.assertTrue("InitializationError: at least one state must be an" +
                        " end_state" in str(context.exception))


if __name__ == '__main__':
    rospy.init_node('test_state_machine')
    rostest.rosrun('letter_learning_interaction', 'test_state_machine',
                   TestStateMachine, sys.argv)                        
