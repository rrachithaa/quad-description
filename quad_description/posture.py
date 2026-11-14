import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

LEGS = ('fl', 'fr', 'rl', 'rr')
JOINTS = [f'{leg}_{j}_joint' for leg in LEGS for j in ('hip', 'thigh', 'knee')]

# (hip, thigh, knee) per leg, in the order fl, fr, rl, rr
POSES = {
    'stand': [0.0, 0.7, -1.4] * 4,
    'lie':   [0.0, 1.3, -2.5] * 4,
    'sit':   [0.0, 0.5, -1.0] * 2 + [0.0, 1.3, -2.5] * 2,
}


def main():
    pose = sys.argv[1] if len(sys.argv) > 1 else 'stand'
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
    if pose not in POSES:
        print(f'Unknown pose "{pose}". Choose from: {", ".join(POSES)}')
        return

    rclpy.init()
    node = Node('posture')
    pub = node.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

    t0 = time.time()
    while pub.get_subscription_count() == 0 and time.time() - t0 < 10.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        print('No controller is listening. Is Gazebo running with the controllers active?')
        node.destroy_node()
        rclpy.shutdown()
        return

    msg = JointTrajectory()
    msg.joint_names = JOINTS
    point = JointTrajectoryPoint()
    point.positions = POSES[pose]
    point.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
    msg.points = [point]
    pub.publish(msg)
    print(f'Sent pose "{pose}" over {secs:.1f} s')
    rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
