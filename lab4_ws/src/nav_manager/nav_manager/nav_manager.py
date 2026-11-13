#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from enum import Enum
from rclypy.action import ActionClient
from nav2_masgs.action import NavigateToPose

# An enum allows you to use words instead of arbitrary
# numbers to represent your states
class State(Enum):
    INIT = 1
    NAVIGATE = 2
    PICK = 3
    PLACE = 4
    # You can add other states to represent other behaviors
    DONE = 5 # Temporarily added NEXT as a state

class NavManager(Node):
    def __init__(self):
        super().__init__('nav_manager')
        self.state = State.INIT # Initialize internal variable to represent state

        # The timer calls the timer_callback function 10 times a second.
        self.timer = self.create_timer(0.1, self.timer_callback)
        # Add subscribers, publishers, action clients, etc. in __init__

        # Can also declare some variables as parameters that can be called
        # from a launch file or the command line
        self.declare_parameter('goal_position', [0.0, 0.0, 0.0])
        self.declare_parameter('block_position', [0.2, 0.0, 0.0])

        goal_pos = self.get_parameter('goal_position').value
        block_pos = self.get_parameter('block_position').value

        # Keep track of whether bot is currently busy
        self.BUSY = False

        # Can use this finished variable to end the node once it is done
        self.finished = False

    # The timer callback handles all logic of what to do when
    def timer_callback(self):
        # Set state to BUSY while something is happening
        if self.BUSY:
            # If currently BUSY, do nothing
            return

        # Could just initialize your state to NAVIGATE
        # but this demonstrates state switching
        elif self.state is State.INIT:
            self.state = State.NAVIGATE
            self.BUSY = False

        elif self.state is State.NAVIGATE:
            self.send_nav_goal()
            self.BUSY = True

        elif self.state is State.DONE:
            # Mark as finished and quit code
            self.finished = True

        # Include any other states in here that you wish
        # Timer callback should just handle switching between
        # other functionality

    def send_nav_goal(self):
        # Send goal to navigation action server in this function

        # After navigation to target, set the state to whatever comes next
        # Manager may not be done after sending nav goal 
        self.state = State.DONE

def main():
    rclpy.init()
    node = NavManager()
    while rclpy.ok() and not node.finished:
        rclpy.spin_once(node, timeout_sec=0.1)
    rclpy.shutdown()
