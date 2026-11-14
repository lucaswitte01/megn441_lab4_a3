import rclpy
import math
import time
from rclpy.node import Node
from rosbot_msgs.msg import IK, Num
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rosbot_msgs.action import Pick, Place

## Pick / place settings ##
# pitch: 90 = gripper pointing straight down (this node's convention)
GRAB_OPEN = 0.2      # lower = wider
GRAB_CLOSED = 0.75
HOVER = 0.06         # m above the block before descending / after lifting
PICK_PITCH = 90.0
ROLL = 0.0
# Stow pose from the lab handout. Check the pitch sign against this node's
# convention (90 = down, -90 = up) the first time you run it.
STOW_XYZ = [0.1, 0.0, 0.24]
STOW_PITCH = 45.0
MOVE_TIME = 1.5      # s for big moves
SLOW_TIME = 1.0      # s for the final approach / lift
GRIP_TIME = 0.8      # s to open or close the gripper
SETTLE = 0.2         # extra wait after each move

## Joint Links ##
# l0 refers to the height of the arm base wrt base_link
l0 = 0.05 + 0.0654868 + 0.0338648 + 0.0772047
l1 = 0.130
l2 = 0.130
l3 = 0.055
tool_link = 0.117
lengths = [l0, l1, l2, l3+tool_link, 0]

arm_ids = [1, 2, 3, 4, 5, 10]
# Only joint 2 is limited to protect the screen
min_pulses = [0, 125, 0, 0, 0]
max_pulses = [1000, 775, 1000, 1000, 1000]
min_angles = [-120, -90, -120, -120, -120]
max_angles = [120, 66, 120, 120, 120]

joint_names = ['joint 1 (base yaw)', 'joint 2 (shoulder)', 'joint 3 (elbow)',
               'joint 4 (wrist pitch)', 'joint 5 (wrist roll)']
# Which sent parameters decide each joint's angle (used in error messages)
joint_inputs = ['x, y', 'x, y, z, pitch', 'x, y, z, pitch', 'x, y, z, pitch', 'roll']

# All joints operate in the range -120 to 120 with a pulse width range of 0 to 1000
def angle_to_pulse(angle):
    pulse = 500 + angle / 0.24
    return pulse

def pulse_to_angle(pulse):
    angle = (pulse - 500) * 0.24
    return angle

class IKNode(Node):
    def __init__(self):
        super().__init__('ik_node')
        self.sub = self.create_subscription(IK, 'arm_control/arm_assignment', self.assignment_callback, 10)
        self.pub = self.create_publisher(Num, 'arm_control/arm_config', 10)
        #create action servers
        #self = object, Pick = name of the message, pick = name, execute_pick = registering name of function
        # Reentrant group + MultiThreadedExecutor (see main) so the arm_assignment
        # subscriber keeps working while a pick/place is sleeping between moves
        self.cb_group = ReentrantCallbackGroup()
        self.pick_server = ActionServer(self, Pick, 'pick', self.execute_pick,
                                        callback_group=self.cb_group)
        self.place_server = ActionServer(self, Place, 'place', self.execute_place,
                                         callback_group=self.cb_group)

    def check_inputs(self, x, y, z, roll, pitch, duration):
        params = {'x': x, 'y': y, 'z': z, 'roll': roll, 'pitch': pitch}
        for name, value in params.items():
            if not math.isfinite(value):
                raise ValueError(f'{name}={value} is not a valid number')
        if not (min_angles[4] <= roll <= max_angles[4]):
            raise ValueError(f'roll={roll:.1f} deg is out of range '
                             f'(must be {min_angles[4]} to {max_angles[4]} deg)')
        if not (-90 <= pitch <= 120):
            raise ValueError(f'pitch={pitch:.1f} deg is out of range '
                             f'(must be -90 (pointing up) to 90 (pointing down) deg)')
        if isinstance(duration, (int, float)) and duration < 0:
            raise ValueError(f'duration={duration} must not be negative')

    def get_theta_1(self, x, y):
        yaw = math.atan2(y, x)
        yaw = yaw * (180 / math.pi)
        return yaw

    def get_pos_4(self, x, y, z, pitch):
        pitch = pitch * (math.pi / 180)
        r = math.sqrt(x**2 + y**2)
        j4r = r - ((l3 + tool_link) * math.cos(pitch))
        j4z = z + ((l3 + tool_link) * math.sin(pitch)) - l0
        return (j4r, j4z)

    def get_theta_3(self, x, y, z, pitch):
        pos_4r, pos_4z = self.get_pos_4(x,y,z,pitch)
        le = math.sqrt(pos_4r**2 + pos_4z**2)
        if le > (l1 + l2):
            raise ValueError(f'target is out of reach (x, y, z, pitch): wrist would be {le:.3f} m '
                             f'from the shoulder, max is {l1 + l2:.3f} m. Move the target closer '
                             f'or change pitch')
        if le <= abs(l1 - l2):
            raise ValueError(f'target is too close to the shoulder (x, y, z, pitch): wrist would be '
                             f'{le:.3f} m from it')
        theta_3 = 2 * math.atan2(math.sqrt(((l1+l2)**2 - (le**2)) / (le**2 - (l1 - l2)**2)), 1)
        return theta_3   # radians, size of elbow bend

    def get_theta_2(self, x, y, z, pitch):
        pos_4r, pos_4z = self.get_pos_4(x,y,z,pitch)
        theta_3 = self.get_theta_3(x,y,z,pitch)
        # '+' = elbow up; elbow down needs joint 2 below horizontal for low targets
        theta_2 = math.atan2(pos_4z, pos_4r) + math.atan2(l2 * math.sin(theta_3), l1 + (l2 * math.cos(theta_3)))
        return theta_2   # radians, link 1 angle above horizontal

    def get_theta_4(self, x, y, z, pitch):
        theta_2 = self.get_theta_2(x,y,z,pitch) * (180 / math.pi)
        theta_3 = self.get_theta_3(x,y,z,pitch) * (180 / math.pi)
        # Joint 4 is relative to link 2, so no 90 offset. Elbow-up bend is -theta_3.
        theta_4 = -pitch - theta_2 + theta_3
        return theta_4

    def get_theta_5(self, roll):
        return roll

    def convert_to_pulse(self, x, y, z, roll, pitch, grab):
        theta_5 = self.get_theta_5(roll)
        theta_4 = self.get_theta_4(x,y,z,pitch)
        # Elbow-up bends the opposite way to theta_3's positive value
        theta_3 = -self.get_theta_3(x,y,z,pitch) * (180 / math.pi)
        # Servo 0 is straight up, theta_2 is measured from horizontal
        theta_2 = self.get_theta_2(x,y,z,pitch) * (180 / math.pi) - 90
        theta_1 = self.get_theta_1(x, y)

        thetas = [theta_1, theta_2, theta_3, theta_4, theta_5]
        pulses = []
        for i in range(5):
            if not (min_angles[i] <= thetas[i] <= max_angles[i]):
                raise ValueError(f'{joint_names[i]} would need {thetas[i]:.1f} deg, limit is '
                                 f'{min_angles[i]} to {max_angles[i]} deg. Change {joint_inputs[i]}')
            pulse = int(round(angle_to_pulse(thetas[i])))
            pulses.append(min(max(pulse, min_pulses[i]), max_pulses[i]))
        pulses.append(int(round(grab * 650)))
        return pulses

    def move(self, xyz, roll, pitch, grab, duration):
        """Run IK and publish one arm command. Raises ValueError if unreachable."""
        x, y, z = xyz
        self.check_inputs(x, y, z, roll, pitch, duration)
        pwms = self.convert_to_pulse(x, y, z, roll, pitch, grab)
        output = Num()
        output.ids = arm_ids
        output.pwm = pwms
        output.duration = float(duration)
        self.pub.publish(output)

    def assignment_callback(self, msg):
        x, y, z = msg.xyz[0], msg.xyz[1], msg.xyz[2]
        try:
            self.move([x, y, z], msg.roll, msg.pitch, msg.grab, msg.duration)
        except ValueError as e:
            self.get_logger().warn(f'Rejected target x={x:.3f}, y={y:.3f}, z={z:.3f}, '
                                   f'roll={msg.roll:.1f}, pitch={msg.pitch:.1f}: {e}')
        except Exception as e:
            self.get_logger().error(f'IK failed unexpectedly: {e!r}')

    def run_steps(self, goal_handle, steps, action_type):
        """Do each (stage name, xyz, pitch, grab, duration) in order, waiting for each.
        Returns the action Result after calling succeed() or abort()."""
        result = action_type.Result()
        feedback = action_type.Feedback()
        for stage, xyz, pitch, grab, duration in steps:
            feedback.stage = stage
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f'{stage}: xyz={xyz}, pitch={pitch}, grab={grab}')
            try:
                self.move(xyz, ROLL, pitch, grab, duration)
            except ValueError as e:
                self.get_logger().error(f'{stage} failed: {e}')
                goal_handle.abort()
                result.message = f'{stage} failed: {e}'
                return result
            # Topics give no "arrived" signal, so wait out the move's duration
            time.sleep(duration + SETTLE)
        goal_handle.succeed()
        result.message = 'done'
        return result

    def execute_pick(self, goal_handle):
        x, y, z = goal_handle.request.xyz
        self.get_logger().info(f'Pick target: x={x}, y={y}, z={z}')
        block = [x, y, z]
        above = [x, y, z + HOVER]
        steps = [
            ('open above block',  above,    PICK_PITCH, GRAB_OPEN,   MOVE_TIME),
            ('lower to block',    block,    PICK_PITCH, GRAB_OPEN,   SLOW_TIME),
            ('close gripper',     block,    PICK_PITCH, GRAB_CLOSED, GRIP_TIME),
            ('lift block',        above,    PICK_PITCH, GRAB_CLOSED, SLOW_TIME),
            ('stow with block',   STOW_XYZ, STOW_PITCH, GRAB_CLOSED, MOVE_TIME),
        ]
        return self.run_steps(goal_handle, steps, Pick)

    def execute_place(self, goal_handle):
        x, y, z = goal_handle.request.xyz
        self.get_logger().info(f'Place target: x={x}, y={y}, z={z}')
        target = [x, y, z]
        above = [x, y, z + HOVER]
        steps = [
            ('move above target', above,    PICK_PITCH, GRAB_CLOSED, MOVE_TIME),
            ('lower block',       target,   PICK_PITCH, GRAB_CLOSED, SLOW_TIME),
            ('release block',     target,   PICK_PITCH, GRAB_OPEN,   GRIP_TIME),
            ('back off',          above,    PICK_PITCH, GRAB_OPEN,   SLOW_TIME),
            ('stow empty',        STOW_XYZ, STOW_PITCH, GRAB_OPEN,   MOVE_TIME),
        ]
        return self.run_steps(goal_handle, steps, Place)



def main():
    rclpy.init()
    ik_node = IKNode()
    executor = MultiThreadedExecutor()
    executor.add_node(ik_node)
    executor.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
