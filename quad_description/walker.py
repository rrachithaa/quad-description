import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from quad_description.gait import KNEE_LIM, THIGH_LIM, TROT, clamp, foot_xz
from quad_description.kinematics import leg_ik
from quad_description.posture import JOINTS, LEGS

TRACK = 0.28  # lateral distance between left and right feet (m)


def compute_targets(phase, scale, vx, wz, period, height, lift, max_step):
    """Joint targets for all 12 joints from a body velocity command (vx m/s, wz rad/s)."""
    v_left = vx - wz * TRACK / 2.0
    v_right = vx + wz * TRACK / 2.0
    side_v = {'fl': v_left, 'rl': v_left, 'fr': v_right, 'rr': v_right}
    out = []
    for leg in LEGS:
        # in a trot the body advances 2*step per gait period, so step = v * period / 2
        step = clamp(side_v[leg] * period / 2.0, (-max_step, max_step)) * scale
        x, z = foot_xz(phase + TROT[leg], height, step, lift * scale)
        th, kn = leg_ik(x, z)
        out += [0.0, clamp(th, THIGH_LIM), clamp(kn, KNEE_LIM)]
    return out


class Walker(Node):
    def __init__(self):
        super().__init__('walker')
        self.declare_parameter('period', 0.8)
        self.declare_parameter('height', 0.28)
        self.declare_parameter('lift', 0.05)
        self.declare_parameter('max_step', 0.08)
        self.declare_parameter('lead', 0.01)
        self.declare_parameter('timeout', 0.5)
        self.pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.vx, self.wz = 0.0, 0.0
        self.last_cmd = -1e9
        self.phase, self.scale = 0.0, 0.0
        self.dt = 0.02
        self.create_timer(self.dt, self.tick)
        self.get_logger().info('Walker ready: publish geometry_msgs/Twist on /cmd_vel')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_cmd(self, m):
        self.vx, self.wz = m.linear.x, m.angular.z
        self.last_cmd = self.now()

    def tick(self):
        p = lambda k: self.get_parameter(k).value
        period = p('period')
        moving = abs(self.vx) > 1e-3 or abs(self.wz) > 1e-3
        active = moving and (self.now() - self.last_cmd) < p('timeout')
        rate = self.dt / period                       # ramp in or out over one gait period
        self.scale += clamp((1.0 if active else 0.0) - self.scale, (-rate, rate))
        self.phase = (self.phase + self.dt / period) % 1.0
        positions = compute_targets(self.phase, self.scale, self.vx, self.wz,
                                    period, p('height'), p('lift'), p('max_step'))
        msg = JointTrajectory()
        msg.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = positions
        lead = p('lead')
        pt.time_from_start = Duration(sec=int(lead), nanosec=int((lead % 1) * 1e9))
        msg.points = [pt]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = Walker()
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
