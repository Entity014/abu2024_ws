#!/usr/bin/env python3
import rclpy
from rclpy import qos, Parameter
from rclpy.node import Node
from numpy import linspace, inf
from math import sin
from sensor_msgs.msg import LaserScan


class ScanFilter(Node):
    def __init__(self):
        super().__init__("scan_filter_node")
        self.pub = self.create_publisher(
            LaserScan, "/scan_filtered", qos_profile=qos.qos_profile_system_default
        )
        self.sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_filter_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )

        self.width = 1
        # self.extent = self.width / 2.0
        self.extent = inf
        self.get_logger().info(
            "Publishing the filtered_scan topic. Use RViz to visualize."
        )

        self.declare_parameters(
            "",
            [
                ("min_angle", Parameter.Type.DOUBLE),
                ("max_angle", Parameter.Type.DOUBLE),
            ],
        )

    def scan_filter_callback(self, msg):
        min_angle = self.get_parameter("min_angle").get_parameter_value().double_value
        max_angle = self.get_parameter("max_angle").get_parameter_value().double_value
        angles = linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        points = [
            (
                r * sin(theta)
                if (
                    (theta < 1.1 and theta > -1.25)
                    or (theta < -1.7 and theta > -2.605)
                    or (theta < 2.43 and theta > 1.5)
                    or (theta < 3.15 and theta > 2.83)
                    or (theta < -3.065 and theta > -3.15)
                )
                else inf
            )
            for r, theta in zip(msg.ranges, angles)
        ]
        # points2 = [
        #     r * sin(theta) if (theta < -1.7 and theta > -2.5) else inf
        #     for r, theta in zip(msg.ranges, angles)
        # ]
        # points3 = [
        #     r * sin(theta) if (theta < 2.5 and theta > 1.7) else inf
        #     for r, theta in zip(msg.ranges, angles)
        # ]
        new_ranges = [
            r if abs(y) < self.extent else inf for r, y in zip(msg.ranges, points)
        ]
        msg.ranges = new_ranges
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    scan_filter = ScanFilter()
    rclpy.spin(scan_filter)
    scan_filter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
