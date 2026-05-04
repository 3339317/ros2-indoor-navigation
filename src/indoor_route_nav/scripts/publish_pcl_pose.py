#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped


def yaw_deg_to_quaternion(yaw_deg):
    yaw = math.radians(yaw_deg)
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(yaw * 0.5),
        "w": math.cos(yaw * 0.5),
    }


class PclPosePublisher(Node):
    def __init__(self, args):
        super().__init__("pcl_pose_test_publisher")
        self.args = args
        self.pub = self.create_publisher(PoseWithCovarianceStamped, args.topic, 10)
        self.timer = self.create_timer(1.0 / args.rate, self.publish_pose)
        self.get_logger().info(
            f"Publishing {args.topic}: x={args.x:.2f}, y={args.y:.2f}, "
            f"z={args.z:.2f}, yaw={args.yaw:.1f}, frame={args.frame_id}, rate={args.rate:.1f}Hz"
        )

    def publish_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.frame_id

        msg.pose.pose.position.x = float(self.args.x)
        msg.pose.pose.position.y = float(self.args.y)
        msg.pose.pose.position.z = float(self.args.z)

        q = yaw_deg_to_quaternion(self.args.yaw)
        msg.pose.pose.orientation.x = q["x"]
        msg.pose.pose.orientation.y = q["y"]
        msg.pose.pose.orientation.z = q["z"]
        msg.pose.pose.orientation.w = q["w"]

        cov = [0.0] * 36
        cov[0] = self.args.cov_xy
        cov[7] = self.args.cov_xy
        cov[14] = self.args.cov_z
        cov[35] = self.args.cov_yaw
        msg.pose.covariance = cov

        self.pub.publish(msg)


def parse_args():
    parser = argparse.ArgumentParser(description="Publish a fixed /pcl_pose for testing the web UI.")
    parser.add_argument("--topic", default="/pcl_pose", help="Pose topic, default: /pcl_pose")
    parser.add_argument("--frame-id", default="map", help="Frame id, default: map")
    parser.add_argument("--x", type=float, default=0.0, help="X position")
    parser.add_argument("--y", type=float, default=0.0, help="Y position")
    parser.add_argument("--z", type=float, default=0.0, help="Z position")
    parser.add_argument("--yaw", type=float, default=0.0, help="Yaw angle in degrees")
    parser.add_argument("--rate", type=float, default=5.0, help="Publish rate in Hz")
    parser.add_argument("--cov-xy", type=float, default=0.25, help="X/Y covariance")
    parser.add_argument("--cov-z", type=float, default=0.25, help="Z covariance")
    parser.add_argument("--cov-yaw", type=float, default=0.0685, help="Yaw covariance")
    return parser.parse_args()


def main():
    args = parse_args()
    args.rate = max(0.1, float(args.rate))

    rclpy.init()
    node = PclPosePublisher(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
