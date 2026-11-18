import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class Avoid(Node):
    def __init__(self):
        super().__init__('avoid')
        self.declare_parameter('speed', 0.08)       # forward speed (m/s)
        self.declare_parameter('turn', 0.5)         # turning rate (rad/s)
        self.declare_parameter('stop_dist', 0.6)    # start turning when front is closer than this (m)
        self.declare_parameter('clear_dist', 0.9)   # stop turning once front is clearer than this (m)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.turning = 0  # +1 = turning left, -1 = turning right, 0 = walking forward
        self.get_logger().info('Avoid ready: reading /scan, publishing /cmd_vel')

    @staticmethod
    def sector_min(scan, lo, hi):
        best = scan.range_max
        for i, r in enumerate(scan.ranges):
            a = scan.angle_min + i * scan.angle_increment
            if lo <= a <= hi and math.isfinite(r) and r >= scan.range_min:
                best = min(best, r)
        return best

    def on_scan(self, scan):
        p = lambda k: self.get_parameter(k).value
        front = self.sector_min(scan, -0.35, 0.35)
        left = self.sector_min(scan, 0.35, 1.57)
        right = self.sector_min(scan, -1.57, -0.35)

        if self.turning == 0 and front < p('stop_dist'):
            self.turning = 1 if left > right else -1
        elif self.turning != 0 and front > p('clear_dist'):
            self.turning = 0

        cmd = Twist()
        if self.turning:
            cmd.angular.z = p('turn') * self.turning
        else:
            cmd.linear.x = p('speed')
            if right < 0.5:
                cmd.angular.z = 0.2
            elif left < 0.5:
                cmd.angular.z = -0.2
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = Avoid()
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
