import rclpy
import math
from rclpy.node import Node
from rosbot_msgs.msg import IK, Num

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

    def assignment_callback(self, msg):
        x = msg.xyz[0]
        y = msg.xyz[1]
        z = msg.xyz[2]
        roll = msg.roll
        pitch = msg.pitch
        grab = msg.grab
        duration = msg.duration

        try:
            self.check_inputs(x, y, z, roll, pitch, duration)
            pwms = self.convert_to_pulse(x, y, z, roll, pitch, grab)
        except ValueError as e:
            self.get_logger().warn(f'Rejected target x={x:.3f}, y={y:.3f}, z={z:.3f}, '
                                   f'roll={roll:.1f}, pitch={pitch:.1f}: {e}')
            return
        except Exception as e:
            self.get_logger().error(f'IK failed unexpectedly: {e!r}')
            return
        output = Num()
        output.ids = arm_ids
        output.pwm = pwms
        output.duration = duration
        self.pub.publish(output)

def main():
    rclpy.init()
    ik_node = IKNode()
    rclpy.spin(ik_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
