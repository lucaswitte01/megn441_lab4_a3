import rclpy
from rclpy.node import Node
from ros_robot_controller.ros_robot_controller_sdk import Board
from rosbot_msgs.msg import Num
from rosbot_msgs.srv import Service

PWM_MIN, PWM_MAX = 0, 1000
JOINT2_MIN, JOINT2_MAX = 125, 775
GRIPPER_ID, GRIPPER_MAX = 10, 650

class ArmControl(Node):
    def __init__(self):
        super().__init__('arm_control')
        self.board = Board()
        self.board.enable_reception()
        
        self.arm_sub = self.create_subscription(Num, 'arm_control/arm_config', self.joint_control, 10)
        self.grip_sub = self.create_subscription(Num, 'arm_control/grip_config', self.grip_control, 10)
        self.pose_srv = self.create_service(Service, 'arm_control/arm_pose', self.return_pwm)

        # Subscribe to a forward control topic
        # Subscribe to a gripper control topic

    # Write an arm_control node to receive joint angles
    # and send messages to the Board.

    def clamp(self, servo_id, pwm):
        if servo_id == 2:
            return max(JOINT2_MIN, min(JOINT2_MAX, pwm))
        if servo_id == GRIPPER_ID:
            return max(PWM_MIN, min(GRIPPER_MAX, pwm))
        return max(PWM_MIN, min(PWM_MAX, pwm))

    def send(self, msg):
        if len(msg.ids) != len(msg.pwm):
            self.get_logger().warn('id and pwm lengths are different')
            return
        targets = [(int(i), int(self.clamp(i,p))) for i, p in zip(msg.ids, msg.pwm)]
        self.board.bus_servo_set_position(msg.duration, targets)

    def joint_control(self, msg):
        self.send(msg)

    def grip_control(self, msg):
        self.send(msg)

    def return_pwm(self, request, response):
        response.pwm = []
        for i in request.id:
            pos = self.board.bus_servo_read_position(int(i))
            if isinstance(pos, (list, tuple)):
                pos = pos[0] if pos else None
            if pos is None:
                self.get_logger().warn(f'could not read servo {i}')
                pos = -1
            response.pwm.append(int(pos))
        return response


def main():
    rclpy.init()
    node = ArmControl()
    rclpy.spin(node)
    rclpy.shutdown()
    

if __name__=="__main__":
    main()
