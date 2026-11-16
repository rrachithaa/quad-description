import argparse
import math
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from quad_description.kinematics import leg_ik
from quad_description.posture import JOINTS, LEGS

# Trot: diagonal pairs move together (fl+rr, then fr+rl)
TROT = {'fl': 0.0, 'rr': 0.0, 'fr': 0.5, 'rl': 0.5}
THIGH_LIM = (-1.5, 1.5)
KNEE_LIM = (-2.6, 0.0)


def clamp(v, lim):
    return max(lim[0], min(lim[1], v))


def foot_xz(phase, height, step, lift, duty=0.5, xoff=0.0):
    """Foot target (x forward, z up) relative to the hip for a gait phase in [0, 1)."""
    phase %= 1.0
    if phase < duty:                      # stance: foot on ground, slides backward
        s = phase / duty
        x = step / 2 - step * s
        z = -height
    else:                                 # swing: foot lifts and moves forward
        s = (phase - duty) / (1.0 - duty)
        x = -step / 2 + step * (1 - math.cos(math.pi * s)) / 2
        z = -height + lift * math.sin(math.pi * s)
    return x + xoff, z


def joint_targets(t, period, height, step, lift, xoff, ramp):
    scale = min(1.0, t / ramp) if ramp > 0 else 1.0
    out = []
    for leg in LEGS:
        x, z = foot_xz(t / period + TROT[leg], height, step * scale, lift * scale, xoff=xoff)
        th, kn = leg_ik(x, z)
        out += [0.0, clamp(th, THIGH_LIM), clamp(kn, KNEE_LIM)]
    return out


def send(pub, positions, secs):
    msg = JointTrajectory()
    msg.joint_names = JOINTS
    p = JointTrajectoryPoint()
    p.positions = positions
    p.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
    msg.points = [p]
    pub.publish(msg)


def main():
    ap = argparse.ArgumentParser(description='Trot gait generator')
    ap.add_argument('--period', type=float, default=0.8, help='seconds per full gait cycle')
    ap.add_argument('--height', type=float, default=0.28, help='body height (m)')
    ap.add_argument('--step', type=float, default=0.0, help='step length (m); 0 = trot in place')
    ap.add_argument('--lift', type=float, default=0.05, help='foot lift (m)')
    ap.add_argument('--xoff', type=float, default=0.0, help='foot x offset (m)')
    ap.add_argument('--duration', type=float, default=8.0, help='seconds to run')
    ap.add_argument('--lead', type=float, default=0.01, help='time_from_start of each streamed setpoint (s)')
    a = ap.parse_args(remove_ros_args(args=sys.argv)[1:])

    rclpy.init()
    node = Node('gait')
    pub = node.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

    t0 = time.time()
    while pub.get_subscription_count() == 0 and time.time() - t0 < 10.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        print('No controller is listening. Is Gazebo running with the controllers active?')
        node.destroy_node()
        rclpy.shutdown()
        return

    stand = []
    th, kn = leg_ik(a.xoff, -a.height)
    for _ in LEGS:
        stand += [0.0, th, kn]
    send(pub, stand, 2.0)
    print('Moving to start pose...')
    time.sleep(3.0)

    print(f'Trotting: period {a.period}s, step {a.step} m, lift {a.lift} m for {a.duration}s (Ctrl+C to stop)')
    start = time.time()
    try:
        while time.time() - start < a.duration:
            t = time.time() - start
            send(pub, joint_targets(t, a.period, a.height, a.step, a.lift, a.xoff, a.period), a.lead)
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        send(pub, stand, 1.5)
        rclpy.spin_once(node, timeout_sec=0.3)
        print('Back to standing pose.')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
