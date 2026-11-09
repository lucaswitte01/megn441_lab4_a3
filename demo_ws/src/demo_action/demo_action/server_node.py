import numpy as np
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from demo_msgs.action import Waypoint
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class ServerNode(Node):
    def __init__(self):
        super().__init__('server_node')
        group = ReentrantCallbackGroup()
        self.serv = ActionServer(self, 
                                 Waypoint, 
                                 'waypoint', 
                                 self.action_callback, 
                                 goal_callback=self.goal_callback,
                                 cancel_callback=self.cancel_callback,
                                 callback_group=group)
        self.pose_sub = self.create_subscription(Pose, 
                                                 'turtle1/pose', 
                                                 self.pose_callback, 10,
                                                callback_group=group)
        self.vel_pub = self.create_publisher(Twist, 
                                             'turtle1/cmd_vel', 10)

        # Starting pose, approximately
        self.x = 5.5
        self.y = 5.5
        self.theta = 0.0

        self.declare_parameter("Kp", 5.0)
        self.declare_parameter("Kth", 5.0)
        self.declare_parameter("max_time", 5.0)
        self.Kp = self.get_parameter("Kp").value
        self.Kth = self.get_parameter("Kth").value
        self.max_time = self.get_parameter("max_time").value

    def pose_callback(self, msg):
        # Update pose with subscriber
        # Assigns message content to internal variable
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta

    def calc_error(self, target):
        # Utility error calculation function
        error = np.zeros((3,))
        # error[0] is x error
        # error[1] is y error
        error[0] = target.x - self.x
        error[1] = target.y - self.y

        # error[2] is theta error
        ang_target = np.arctan2(error[1], error[0]) # Direction to target
        error[2] = ang_target - self.theta
        error[2] = ((error[2]+np.pi) % (2*np.pi)) - np.pi # Make sure theta error is in [-pi, pi]
        return error

    def move(self, error):
        # Utility P-control function
        self.Kp = self.get_parameter("Kp").value
        self.Kth = self.get_parameter("Kth").value
        dist_remaining = np.sqrt(error[0]**2 + error[1]**2)
        cmd_msg = Twist()
        if (dist_remaining < 5e-2):
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0
            self.vel_pub.publish(cmd_msg)
            return True # done!
        elif (np.abs(error[2]) < 0.1):
            cmd_msg.linear.x = self.Kp*dist_remaining
            cmd_msg.angular.z = self.Kth*error[2]
            self.vel_pub.publish(cmd_msg)
        else:
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = self.Kth*error[2]
            self.vel_pub.publish(cmd_msg)
        return False # not done

    def goal_callback(self, goal_request):
        # OPTIONAL but good practice: Accept or reject a client request to begin an action
        x = goal_request.target.x
        y = goal_request.target.y
        self.get_logger().info(f'Received goal request, ({x},{y})')
        if (x > 11 or y > 11 or x < 0 or y < 0):
            self.get_logger().warn('Target (x, y) should be between 0 and 11')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    def action_callback(self, goal_handle):
        target = goal_handle.request.target
        
        start_time = self.get_clock().now().nanoseconds
        run_time = (self.get_clock().now().nanoseconds - start_time)*1e-9

        done = False

        self.max_time = self.get_parameter("max_time").value
        while (not done):
            error = self.calc_error(target)
            done = self.move(error)

            feedback_msg = Waypoint.Feedback()
            feedback_msg.distance = np.sqrt(error[0]**2+error[1]**2)
            feedback_msg.angle = error[2]
            goal_handle.publish_feedback(feedback_msg)

            time.sleep(0.05)
            run_time = (self.get_clock().now().nanoseconds - start_time)*1e-9
            if (run_time > self.max_time):
                goal_handle.abort()
                break
            if (goal_handle.is_cancel_requested):
                goal_handle.canceled()
                cmd_msg = Twist()
                cmd_msg.linear.x = 0.0
                cmd_msg.angular.z = 0.0
                self.vel_pub.publish(cmd_msg)
                break
            
        if done:
            goal_handle.succeed()
        output_msg = Waypoint.Result()
        output_msg.position.x = self.x
        output_msg.position.y = self.y
        return output_msg

def main():
    rclpy.init()
    node = ServerNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()