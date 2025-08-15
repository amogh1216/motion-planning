#!/usr/bin/env python3
# adapted from https://www.youtube.com/watch?v=uls-WmxRiTw  @https://github.com/AtsushiSakai 

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped, TwistStamped
import numpy as np
import math

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.path = []
        self.lookahead_distance = 1  # meters
        self.k = 0.1  # lookahead gain
        self.Lfc = 1.5  # min lookahead
        self.L = 0.36  # wheelbase (tune as needed)
        self.target_speed = 0.8  # m/s
        self.pose = None
        self.path_received = False
        self.target_ind = 0

        self.path_sub = self.create_subscription(
            Float64MultiArray, '/path', self.path_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/get_pose', self.pose_callback, 10)
        self.cmd_pub = self.create_publisher(
            TwistStamped, '/rwd_diff_controller/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.control_loop)

    def path_callback(self, msg):
        arr = np.array(msg.data)
        if arr.size % 2 != 0 or arr.size == 0:
            self.get_logger().warn('Received malformed path array')
            return
        self.path = arr.reshape((-1, 2))
        self.path_received = True
        self.target_ind = 0

    def pose_callback(self, msg):
        self.pose = msg

    def control_loop(self):
        if not self.path_received or self.pose is None or len(self.path) < 2:
            return
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        q = self.pose.pose.orientation
        # Convert quaternion to yaw
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Log current pose
        self.get_logger().info(f"Current pose: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")

        # Find nearest point
        dists = np.linalg.norm(self.path - np.array([x, y]), axis=1)
        nearest_ind = np.argmin(dists)
        nearest_point = self.path[nearest_ind]
        self.get_logger().info(f"""Nearest path point: x={nearest_point[0]:.3f}, y={nearest_point[1]:.3f}, 
                               index={nearest_ind}/{len(self.path)-1}""")

        Lf = self.k * self.target_speed + self.Lfc
        # Look ahead to find target point
        ind = nearest_ind
        total_dist = 0.0
        while ind < len(self.path) - 1:
            seg = np.linalg.norm(self.path[ind+1] - self.path[ind])
            total_dist += seg
            if total_dist > Lf:
                break
            ind += 1
        target = self.path[ind]
        # If at the end of the path, stop
        at_goal = (ind >= len(self.path) - 1) and (np.linalg.norm([x - target[0], y - target[1]]) < 0.2)
        # Transform target to robot frame
        dx = target[0] - x
        dy = target[1] - y
        target_y = math.sin(-yaw) * dx + math.cos(-yaw) * dy
        # Pure pursuit steering
        alpha = math.atan2(target_y, Lf)
        v = 0.0 if at_goal else self.target_speed
        omega = 0.0 if at_goal else 2 * v * math.sin(alpha) / Lf
        if at_goal:
            self.get_logger().info("Reached end of path. Stopping robot.")
        # Publish TwistStamped
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = v
        cmd.twist.angular.z = omega
        self.cmd_pub.publish(cmd)
        # Log published command
        self.get_logger().info(f"Published Vel: linear.x={v:.3f}, angular.z={omega:.3f}")


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
