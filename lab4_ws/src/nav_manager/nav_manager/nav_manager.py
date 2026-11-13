#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from enum import Enum
from rclypy.action import ActionClient
from nav2_masgs.action import NavigateToPose
from math import cos, radians, sin
from action_msgs.msg import GoalStatus
# An enum allows you to use words instead of arbitrary
# numbers to represent your states
class State(Enum):
    INIT = 1
    NAVIGATE = 2
    PICK = 3
    PLACE = 4
    # You can add other states to represent other behaviors
    DONE = 5 # Temporarily added NEXT as a state
    FAILED = 6

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

        self.goal_pos = self.get_parameter('goal_position').value
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
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
            if self.nav_client.server_is_ready():
                self.send_nav_goal()

        elif self.state in (State.DONE, State.FAILED):
            # Mark as finished and quit code
            self.finished = True

        # Include any other states in here that you wish
        # Timer callback should just handle switching between
        # other functionality

    def send_nav_goal(self):
        # Send goal to navigation action server in this function
        x, y, yaw_deg = self.goal_pos
        yaw = radians(yaw_deg)

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = sin(yaw/2)
        goal.pose.pose.position.w = cos(yaw/2)
        
        # After navigation to target, set the state to whatever comes next
        self.BUSY = True
        future = self.nav_client.send_goal_async(goal,feedback_callback=self.feedback_callback,)
        future.add_done_callback(self.on_goal_response)
        self.get_logger().info('Sent navigation goal')


    def on_goal_response(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Nav2 rejected the goal')
            self.state = State.FAILED
            self.BUSY = False
            return
        self.get_logger().info('Nav2 goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_nav_result)

         # Manager may not be done after sending nav goal 
    def on_nav_result(self, future):
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Destination reached')
            self.state = State.DONE
        else:
            self.get_logger().error(f'Navigation ended with status {status}')
            self.state = State.FAILED

        self.BUSY = False
    
    def feedback_callback(self, feedback_msg):
        distance = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f'Distance remaining: {distance:.2f} m ')


def main():
    rclpy.init()
    node = NavManager()
    while rclpy.ok() and not node.finished:
        rclpy.spin_once(node, timeout_sec=0.1)
    rclpy.shutdown()
