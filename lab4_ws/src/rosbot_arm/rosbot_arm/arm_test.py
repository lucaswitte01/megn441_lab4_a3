#!/usr/bin/env python3

from ros_robot_controller.ros_robot_controller_sdk import Board
import rclpy
import time

# Utility function to quickly spin a node
# No need to edit
def reliable_spin(node, pub, msg, max_time=2.0):
    timeout = time.time() + max_time
    while pub.get_subscription_count() < 1 and time.time() < timeout:
        rclpy.spin_once(node, timeout_sec=0.05)

    pub.publish(msg)
    node.get_logger().info(f"Publishing xyz={xyz}, roll={roll}, pitch={pitch}")
    for _ in range(5):
        rclpy.spin_once(node, timeout_sec=0.05)



## Setup ##
# All joints go from -120 to 120 degrees
# Corresponding to a pulse width of 0 to 1000 ms
# Pulse commands are in units of ms

# List of joint (id, command) pairs
targets = [(1, 500),
           (2, 200), # KEEP JOINT 2 BETWEEN 125 AND 775
           (3, 300),
           (4, 350),
           (5, 500),
           (10, 350)] # Gripper joint. Keep below 650

duration = 10.0 # Time to complete motion




## Stage 1: Direct: ##
# Send commands directly to board for each
# joint id, pulling from targets/duration above
def direct():
    board = Board()
    board.enable_reception()
    board.bus_servo_set_position(duration, targets)
    for i in [1, 2, 3, 4, 5, 10]:
        print(i, ":\t", board.bus_servo_read_position(i))



## Stage 2: Forward: ##
# Send commands through your topic for each
# joint id, using targets, duration above

# TODO: Update to your ForwardMsg type
# from rosbot_msgs.msg import ForwardMsg

def forward():
    rclpy.init()
    node = rclpy.create_node('arm_test')
    # TODO: Update ForwardMsg, 'forward_topic' with your type, topic
    forward_pub = node.create_publisher(ForwardMsg, 'forward_topic', 10)
    msg = ForwardMsg()

    # TODO: Assign targets, duration to your ForwardMsg

    reliable_spin(node, forward_pub, msg)



## Stage 3: Inverse ##
# Send command sthrough your inverse topic for target
# xyz, roll, and pitch

# TODO: Update to your InverseMsg type
# from rosbot_msgs.msg import InverseMsg

# End position of arm relative to base_link
xyz = [0.18, 0.02, 0.32]
roll = 0.0 # Relative roll rotation of gripper
pitch = -30.0 # Relative pitch rotation of gripper
# Don't need yaw! It's a 5-DOF arm and yaw gets set based on xyz

def inverse():
    rclpy.init()
    node = rclpy.create_node('arm_test')
    # TODO: Update InverseMsg, 'inverse_topic' with your type, topic
    inverse_pub = node.create_publisher(InverseMsg, 'inverse_topic', 10)
    msg = InverseMsg()

    # TODO: Assign duration, xyz, roll, pitch to your InverseMsg

    reliable_spin(node, inverse_pub, msg)



def main(mode='direct'):
    if mode == 'direct':
        direct()
    elif mode=='forward':
        forward()
    elif mode=='inverse':
        inverse()
    else:
        print('Mode options are direct, forward, or inverse')

if __name__ == '__main__':
    main('direct')
