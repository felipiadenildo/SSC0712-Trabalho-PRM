#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class RobotPathNode(Node):
    def __init__(self):
        super().__init__('robot_path_node')
        self.path = Path()
        self.path.header.frame_id = 'odom'
        self.pub = self.create_publisher(Path, '/robot/actual_path', 10)
        self.create_subscription(Odometry, '/odom_gt', self.odom_cb, 10)

    def odom_cb(self, msg: Odometry):
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self.path.poses.append(ps)
        # opcional: limite o tamanho de self.path.poses para não crescer infinitamente
        self.pub.publish(self.path)

def main(args=None):
    rclpy.init(args=args)
    node = RobotPathNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
