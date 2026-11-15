import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from quad_description.kinematics import leg_ik
from quad_description.posture import JOINTS

THIGH_LIM = (-1.5, 1.5)
KNEE_LIM = (-2.6, 0.0)


def main():
    height = float(sys.argv[1]) if len(sys.argv) > 1 else 0.30
    x_off = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    secs = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0

    thigh, knee = leg_ik(x_off, -height)
    print(f'height {height:.2f} m, foot x-offset {x_off:+.2f} m -> thigh {thigh:+.3f}, knee {knee:+.3f}')
    if not (THIGH_LIM[0] <= thigh <= THIGH_LIM[1] and KNEE_LIM[0] <= knee <= KNEE_LIM[1]):
        print('That is outside the joint limits. Try a height between about 0.14 and 0.36.')
        return

    rclpy.init()
    node = Node('height')
    pub = node.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
    t0 = time.time()
    while pub.get_subscription_count() == 0 and time.time() - t0 < 10.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        print('No controller is listening. Is Gazebo running with the controllers active?')
    else:
        msg = JointTrajectory()
        msg.joint_names = JOINTS
        point = JointTrajectoryPoint()
        point.positions = [0.0, thigh, knee] * 4
        point.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        msg.points = [point]
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.5)
        print('Sent.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
