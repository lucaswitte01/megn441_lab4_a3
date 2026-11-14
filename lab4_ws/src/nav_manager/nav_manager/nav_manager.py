#!/usr/bin/env python3
 
import rclpy
from rclpy.node import Node
from enum import Enum
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from rosbot_msgs.action import Pick, Place
from math import cos, radians, sin
from action_msgs.msg import GoalStatus
 
 
# Order: INIT -> PICK -> NAVIGATE -> PLACE -> DONE
class State(Enum):
    INIT = 1
    PICK = 2
    NAVIGATE = 3
    PLACE = 4
    DONE = 5
    FAILED = 6
 
 
class NavManager(Node):
    def __init__(self):
        super().__init__('nav_manager')
        self.state = State.INIT
 
        # The timer calls the timer_callback function 10 times a second.
        self.timer = self.create_timer(0.1, self.timer_callback)
 
        self.declare_parameter('goal_position', [0.0, 0.0, 0.0])
        self.declare_parameter('block_position', [0.2, 0.0, 0.0])
 
        self.goal_pos = self.get_parameter('goal_position').value
        self.block_pos = self.get_parameter('block_position').value
 
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # Names must match the ActionServer names in ik_node ('pick', 'place')
        self.pick_client = ActionClient(self, Pick, 'pick')
        self.place_client = ActionClient(self, Place, 'place')
 
        # Keep track of whether bot is currently busy
        self.BUSY = False
        # State to move to when the current arm action succeeds
        self.next_state = None
 
        # Can use this finished variable to end the node once it is done
        self.finished = False
 
    # The timer callback only decides what to start next.
    # Each action's result callback moves the state forward and clears BUSY.
    def timer_callback(self):
        if self.BUSY:
            return
 
        elif self.state is State.INIT:
            self.state = State.PICK
 
        elif self.state is State.PICK:
            if self.pick_client.server_is_ready():
                self.send_arm_goal(self.pick_client, Pick, State.NAVIGATE)
 
        elif self.state is State.NAVIGATE:
            if self.nav_client.server_is_ready():
                self.send_nav_goal()
 
        elif self.state is State.PLACE:
            if self.place_client.server_is_ready():
                self.send_arm_goal(self.place_client, Place, State.DONE)
 
        elif self.state in (State.DONE, State.FAILED):
            self.finished = True
 
    # ---------- Arm (pick / place) ----------
    def send_arm_goal(self, client, action_type, next_state):
        goal = action_type.Goal()
        goal.xyz = [float(v) for v in self.block_pos]
        self.next_state = next_state
        self.BUSY = True
        future = client.send_goal_async(goal, feedback_callback=self.arm_feedback_callback)
        future.add_done_callback(self.on_arm_goal_response)
        self.get_logger().info(f'Sent {action_type.__name__} goal: {goal.xyz}')
 
    def on_arm_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Arm goal rejected!')
            self.state = State.FAILED
            self.BUSY = False
            return
        goal_handle.get_result_async().add_done_callback(self.on_arm_result)
 
    def on_arm_result(self, future):
        response = future.result()
        if response.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Arm action finished: {response.result.message}')
            self.state = self.next_state
        else:
            self.get_logger().error(f'Arm action failed ({response.status}): '
                                    f'{response.result.message}')
            self.state = State.FAILED
        self.BUSY = False
 
    def arm_feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Arm: {feedback_msg.feedback.stage}')
 
    # ---------- Navigation ----------
    def send_nav_goal(self):
        x, y, yaw_deg = self.goal_pos
        yaw = radians(yaw_deg)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = sin(yaw / 2)
        goal.pose.pose.orientation.w = cos(yaw / 2)
 
        self.BUSY = True
        future = self.nav_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.on_goal_response)
        self.get_logger().info('Sent navigation goal')
 
    def on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 Goal rejected!')
            self.state = State.FAILED
            self.BUSY = False
            return
        self.get_logger().info('Nav2 Goal accepted')
        goal_handle.get_result_async().add_done_callback(self.on_nav_result)
 
    def on_nav_result(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Destination reached')
            self.state = State.PLACE          # was State.DONE
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
 
 
if __name__ == '__main__':
    main()
 

