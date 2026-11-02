"""
block_stacker

1. Move to the look pose (default xyz [0.2, 0.0, 0.1], camera pointing down)
2. Wait until a block is in view, then pick it up
3. Go back to the look pose and wait until another block is in view
   (ignoring the spot the first one came from), then stack the first block on it
4. Return to the look pose

Camera -> arm transform is built from the arm's own pose:
  - tool xyz + pitch from the last IK command ik_node accepted
  - base swivel angle from pwm[0] on /arm_control/arm_config
  - camera mount offsets measured along the gripper
All positions are in ik_node's frame (x/y from the swivel axis, z from base_link).

Every move waits for ik_node to confirm it (arm_config). Pick and place targets
are checked against ik_node's own math first, and the pitch is lowered from 90
if a block is out of reach with the gripper straight down.
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from rosbot_msgs.msg import BlockDetection, IK, Num

# Use ik_node's own math to check targets before sending them
try:
    from rosbot_arm import ik_node as _ik_module
    _IK = _ik_module.IKNode.__new__(_ik_module.IKNode)   # math only, no ROS node
except Exception:   # noqa: BLE001
    _IK = None


def ik_check(xyz, roll, pitch, grab):
    """Return '' if ik_node would accept this target, otherwise the reason."""
    if _IK is None:
        return ''
    x, y, z = xyz
    try:
        _IK.check_inputs(x, y, z, roll, pitch, 1.0)
        _IK.convert_to_pulse(x, y, z, roll, pitch, grab)
    except ValueError as e:
        return str(e)
    return ''


def pulse_to_angle(pulse):
    """Same conversion as ik_node.py: pulse 0-1000 -> -120 to 120 deg."""
    return (pulse - 500) * 0.24


def camera_to_arm_T(tool_r, tool_z, yaw_deg, pitch_deg, cam_behind, cam_above,
                    cam_tilt_deg=0.0):
    """
    4x4 transform from depth_cam_color_optical_frame to the IK frame.

    tool_r, tool_z : IK tool point (radial distance from the swivel axis, height)
    yaw_deg        : base swivel angle, same convention as ik_node (atan2(y, x))
    pitch_deg      : IK pitch, same convention as ik_node (90 = pointing down)
    cam_behind     : camera distance behind the tool point, along the gripper
    cam_above      : camera distance above the gripper axis (camera's top side)
    cam_tilt_deg   : how far the lens points away from the gripper direction,
                     + = tilted toward the camera's top (looks farther out at pitch 90)
    """
    yaw = math.radians(yaw_deg)
    p = math.radians(pitch_deg)
    q = math.radians(pitch_deg - cam_tilt_deg)   # direction the lens actually points

    r_hat = np.array([math.cos(yaw), math.sin(yaw), 0.0])    # out along the arm
    s_hat = np.array([-math.sin(yaw), math.cos(yaw), 0.0])   # arm's left
    z_hat = np.array([0.0, 0.0, 1.0])                        # up

    axis = math.cos(p) * r_hat - math.sin(p) * z_hat   # gripper direction
    top = math.sin(p) * r_hat + math.cos(p) * z_hat    # camera's top side
    tool = tool_r * r_hat + tool_z * z_hat
    cam = tool - cam_behind * axis + cam_above * top

    lens_axis = math.cos(q) * r_hat - math.sin(q) * z_hat
    lens_top = math.sin(q) * r_hat + math.cos(q) * z_hat

    # Optical frame: x = image right, y = image down, z = out of the lens
    T = np.eye(4)
    T[:3, 0] = -s_hat
    T[:3, 1] = -lens_top
    T[:3, 2] = lens_axis
    T[:3, 3] = cam
    return T


def xy_dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


class Block:
    """Detections that belong to the same physical block."""

    def __init__(self, color, p):
        self.color = color
        self.sum = np.array(p, dtype=float)
        self.n = 1

    @property
    def pos(self):
        return self.sum / self.n

    def add(self, p):
        self.sum += p
        self.n += 1

    def __str__(self):
        x, y, z = self.pos
        return f'{self.color} block at ({x:.3f}, {y:.3f}, {z:.3f})'


class BlockStacker(Node):
    def __init__(self):
        super().__init__('block_stacker')

        P = self.declare_parameter
        # Topics
        P('detection_topic', '/block_detector/block_detections')
        P('ik_topic', '/arm_control/arm_assignment')
        P('config_topic', '/arm_control/arm_config')

        # Camera mount relative to the IK tool point, measured along the gripper
        P('cam_behind_tool', 0.10)
        P('cam_above_tool', 0.05)
        P('cam_tilt_deg', 0.0)        # + = lens tilted up from the gripper direction

        # Look pose: the arm waits here and watches for blocks
        P('look_xyz', [0.2, 0.0, 0.1])
        P('look_pitch', 90.0)
        P('look_duration', 3.0)
        P('look_dwell', 1.0)          # s of detections per look
        P('merge_radius', 0.05)       # detections closer than this = same block
        P('min_hits', 3)              # detections needed to count as a block
        P('exclude_radius', 0.06)     # ignore blocks this close to where the 1st was
        P('min_detect_depth', 0.14)   # m, ignores the block held in the gripper
        P('pick_color', '')           # '' = any color

        # Gripping
        P('roll', 0.0)
        P('pitch', 90.0)              # preferred gripping pitch
        P('min_pick_pitch', 30.0)     # lowest pitch allowed when 90 is out of reach
        P('pitch_step', 5.0)
        P('grab_open', 0.2)           # lower = wider
        P('grab_close', 0.75)
        P('move_duration', 1.5)
        P('slow_duration', 2.5)       # descending / placing
        P('grip_duration', 1.0)       # opening / closing the gripper
        P('settle_time', 0.5)

        # Geometry and manual corrections
        P('block_size', 0.03)
        P('hover_height', 0.06)       # m above the grasp / place height
        P('place_clearance', 0.005)   # gap above the lower block when releasing
        P('grasp_z_offset', 0.0)      # + raises the grasp, - lowers it
        P('grasp_offset_x', 0.0)      # m, added to every grip / release position
        P('grasp_offset_y', 0.02)     # (gripper was landing ~2 cm too far toward -y)

        P('dry_run', False)           # find and measure both blocks, don't grip

        self.ik_pub = self.create_publisher(IK, self.p('ik_topic'), 10)
        self.create_subscription(Num, self.p('config_topic'), self.config_callback, 10)
        self.create_subscription(
            BlockDetection, self.p('detection_topic'), self.detection_callback, 10)

        # Arm pose tracking
        self.pending_cmd = None
        self.pending_settle = self.p('settle_time')
        self.arm_pose = None          # (r, z, pitch, yaw)
        self.settle_until = None
        self.config_count = 0

        # Detections
        self.collecting = False
        self.hits = []                # (color, point) collected at the current stop

        # Step queue
        self.queue = []
        self.step = None
        self.step_deadline = None
        self.move_config_count = 0

        # Task state
        self.first = None             # block that was picked up
        self.carry_offset = None      # tool height above the held block's bottom face
        self.holding = False
        self.last_wait_log = None

        self.go_look()
        self.call(lambda: self.look_for('pick'))

        self.start_time = self.get_clock().now()
        self.create_timer(0.05, self.tick)
        self.get_logger().info('block_stacker started')

    def p(self, name):
        return self.get_parameter(name).value

    # ---------- arm pose tracking ----------

    def config_callback(self, msg):
        """ik_node only publishes arm_config when it accepted a command."""
        if self.pending_cmd is None or len(msg.pwm) == 0:
            return
        (x, y, z), pitch = self.pending_cmd
        self.arm_pose = (math.hypot(x, y), z, pitch, pulse_to_angle(msg.pwm[0]))
        wait = float(msg.duration) + self.pending_settle
        self.settle_until = self.get_clock().now() + Duration(seconds=wait)
        self.config_count += 1

    def send_ik(self, xyz, grab, duration, pitch):
        cmd = IK()
        cmd.xyz = [float(v) for v in xyz]
        cmd.roll = float(self.p('roll'))
        cmd.pitch = float(pitch)
        cmd.grab = float(grab)
        cmd.duration = float(duration)
        self.pending_cmd = (cmd.xyz[:3], cmd.pitch)
        self.ik_pub.publish(cmd)

    # ---------- detections ----------

    def detection_callback(self, msg):
        if not self.collecting or self.arm_pose is None:
            return
        if self.settle_until is None or self.get_clock().now() < self.settle_until:
            return
        if msg.position.z < self.p('min_detect_depth'):
            return   # no depth, or the block in the gripper

        r, z, pitch, yaw = self.arm_pose
        T = camera_to_arm_T(r, z, yaw, pitch, self.p('cam_behind_tool'),
                            self.p('cam_above_tool'), self.p('cam_tilt_deg'))
        pt = (T @ np.array([msg.position.x, msg.position.y, msg.position.z, 1.0]))[:3]
        self.hits.append((msg.color, pt))

    def group_hits(self):
        """Group the current stop's detections into blocks."""
        blocks = []
        for color, pt in self.hits:
            for b in blocks:
                if b.color == color and xy_dist(b.pos, pt) < self.p('merge_radius'):
                    b.add(pt)
                    break
            else:
                blocks.append(Block(color, pt))
        return [b for b in blocks if b.n >= self.p('min_hits')]

    # ---------- step queue ----------

    def move(self, name, xyz, grab, duration, required=True, pitch=None, settle=None):
        self.queue.append({'type': 'move', 'name': name, 'xyz': list(xyz),
                           'grab': grab, 'duration': duration, 'required': required,
                           'pitch': self.p('pitch') if pitch is None else pitch,
                           'settle': self.p('settle_time') if settle is None else settle})

    def collect(self, seconds, on_done):
        self.queue.append({'type': 'collect', 'seconds': seconds, 'on_done': on_done})

    def call(self, fn):
        self.queue.append({'type': 'call', 'fn': fn})

    def tick(self):
        now = self.get_clock().now()
        if now - self.start_time < Duration(seconds=1.0):
            return   # let publishers/subscribers connect
        if self.step is None:
            if not self.queue:
                return
            # Commands sent before ik_node has connected are silently lost
            if self.ik_pub.get_subscription_count() == 0:
                self.get_logger().warn(
                    f"Waiting for ik_node to subscribe to {self.p('ik_topic')} "
                    f"(is ik_node running?)", throttle_duration_sec=3.0)
                return
            self.start_step(self.queue.pop(0))
            return
        if now >= self.step_deadline:
            self.finish_step()

    def start_step(self, step):
        now = self.get_clock().now()
        self.step = step
        if step['type'] == 'move':
            x, y, z = step['xyz']
            self.get_logger().info(
                f"{step['name']}: xyz=[{x:.3f}, {y:.3f}, {z:.3f}] "
                f"pitch={step['pitch']:.0f} grab={step['grab']}")
            self.move_config_count = self.config_count
            self.pending_settle = step['settle']
            self.send_ik(step['xyz'], step['grab'], step['duration'], step['pitch'])
            self.step_deadline = now + Duration(seconds=step['duration'] + step['settle'])
        elif step['type'] == 'collect':
            self.hits = []
            self.collecting = True
            self.step_deadline = now + Duration(seconds=step['seconds'])
        else:
            self.step = None
            step['fn']()

    def finish_step(self):
        step = self.step
        self.step = None
        if step['type'] == 'move':
            if self.config_count == self.move_config_count:
                if step.get('tries', 0) < 1:
                    step['tries'] = step.get('tries', 0) + 1
                    self.get_logger().warn(
                        f"'{step['name']}' not confirmed by ik_node, retrying once")
                    self.queue.insert(0, step)
                    return
                msg = (f"'{step['name']}' was not accepted by ik_node (out of reach, "
                       f"or ik_node / arm_control not running). Check the ik_node "
                       f"terminal for the reason")
                if step['required']:
                    self.get_logger().error(msg + '. Stopping.')
                    self.queue.clear()
                    return
                # The arm stayed put; any following measurement still works,
                # since detections use the arm's actual (unchanged) pose
                self.get_logger().warn(msg + '. Skipping.')
        elif step['type'] == 'collect':
            self.collecting = False
            step['on_done']()

    def abort(self, reason):
        self.get_logger().error(reason)
        self.queue.clear()

    # ---------- pitch helpers ----------

    def pitches(self, start):
        pitch = start
        while pitch >= self.p('min_pick_pitch') - 1e-6:
            yield pitch
            pitch -= self.p('pitch_step')

    def find_pitch(self, targets, grabs):
        """Highest pitch (closest to straight down) that reaches every target."""
        if _IK is None:
            return self.p('pitch'), ''
        first_err = ''
        for pitch in self.pitches(self.p('pitch')):
            err = next((e for t in targets for g in grabs
                        if (e := ik_check(t, self.p('roll'), pitch, g))), '')
            if not err:
                return pitch, ''
            first_err = first_err or f'at pitch {pitch:.0f}, {err}'
        return None, first_err

    # ---------- looking ----------

    def grab_now(self):
        return self.p('grab_close') if self.holding else self.p('grab_open')

    def go_look(self, grab=None):
        grab = self.grab_now() if grab is None else grab
        self.move('look pose', self.p('look_xyz'), grab, self.p('look_duration'),
                  pitch=self.p('look_pitch'))

    def look_for(self, purpose):
        """Collect detections at the look pose; repeat until a usable block shows up."""
        self.collect(self.p('look_dwell'), lambda: self.look_done(purpose))

    def usable(self, block, purpose):
        if purpose == 'pick':
            color = self.p('pick_color')
            return not color or block.color == color
        if self.first is not None and xy_dist(block.pos, self.first.pos) < self.p('exclude_radius'):
            return False   # where the first block was picked up from
        return True

    def look_done(self, purpose):
        found = [b for b in self.group_hits() if self.usable(b, purpose)]
        if not found:
            now = self.get_clock().now()
            if self.last_wait_log is None or now - self.last_wait_log > Duration(seconds=5.0):
                self.last_wait_log = now
                what = 'a block to pick up' if purpose == 'pick' else 'a block to stack on'
                self.get_logger().info(f'Waiting for {what} in view...')
            self.look_for(purpose)
            return
        block = max(found, key=lambda b: b.n)
        self.last_wait_log = None
        self.get_logger().info(f'Found {block}')
        if purpose == 'pick':
            self.first = block
            self.plan_pick(block)
        else:
            self.plan_place(block)

    # ---------- pick and place ----------

    def tool_xy(self, x, y):
        """Where to put the tool so it lines up with a block at (x, y)."""
        return x + self.p('grasp_offset_x'), y + self.p('grasp_offset_y')

    def plan_pick(self, block):
        bx, by, top = block.pos
        tx, ty = self.tool_xy(bx, by)
        size = self.p('block_size')
        z_grasp = top - size / 2 + self.p('grasp_z_offset')
        z_hover = z_grasp + self.p('hover_height')
        g_open, g_close = self.p('grab_open'), self.p('grab_close')

        pitch, err = self.find_pitch([[tx, ty, z_hover], [tx, ty, z_grasp]],
                                     [g_open, g_close])
        if pitch is None:
            self.abort(f'{block} is out of reach ({err}). Move it closer to the arm.')
            return
        self.carry_offset = z_grasp - (top - size)

        if self.p('dry_run'):
            self.get_logger().info(
                f'Dry run: would grasp at ({tx:.3f}, {ty:.3f}, {z_grasp:.3f}) '
                f'pitch {pitch:.0f}')
        else:
            move_d, slow_d, grip_d = (self.p('move_duration'), self.p('slow_duration'),
                                      self.p('grip_duration'))
            self.move('above block', [tx, ty, z_hover], g_open, move_d, pitch=pitch)
            self.move('lower to block', [tx, ty, z_grasp], g_open, slow_d, pitch=pitch)
            self.move('close gripper', [tx, ty, z_grasp], g_close, grip_d, pitch=pitch)
            self.move('lift', [tx, ty, z_hover], g_close, slow_d, pitch=pitch)
            self.call(lambda: setattr(self, 'holding', True))
            self.move('lift to look pose', self.p('look_xyz'), g_close,
                      self.p('move_duration'), pitch=self.p('look_pitch'))
        self.call(lambda: self.look_for('place'))

    def plan_place(self, block):
        bx, by, top = block.pos
        tx, ty = self.tool_xy(bx, by)
        z_place = top + self.carry_offset + self.p('place_clearance')
        z_hover = z_place + self.p('hover_height')
        g_open, g_close = self.p('grab_open'), self.p('grab_close')

        pitch, err = self.find_pitch([[tx, ty, z_hover], [tx, ty, z_place]],
                                     [g_open, g_close])
        if pitch is None:
            self.abort(f'{block} is out of reach ({err}). Move it closer to the arm.')
            return

        if self.p('dry_run'):
            self.get_logger().info(
                f'Dry run: would release at ({tx:.3f}, {ty:.3f}, {z_place:.3f}) '
                f'pitch {pitch:.0f}')
        else:
            move_d, slow_d, grip_d = (self.p('move_duration'), self.p('slow_duration'),
                                      self.p('grip_duration'))
            self.move('above stack', [tx, ty, z_hover], g_close, move_d, pitch=pitch)
            self.move('lower onto stack', [tx, ty, z_place], g_close, slow_d, pitch=pitch)
            self.move('open gripper', [tx, ty, z_place], g_open, grip_d, pitch=pitch)
            self.call(lambda: setattr(self, 'holding', False))
            self.move('clear stack', [tx, ty, z_hover], g_open, slow_d, pitch=pitch)
        self.go_look(grab=g_open if not self.p('dry_run') else None)
        self.call(lambda: self.get_logger().info('Done'))


def main(args=None):
    rclpy.init(args=args)
    node = BlockStacker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
