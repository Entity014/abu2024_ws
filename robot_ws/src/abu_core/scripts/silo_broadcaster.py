#!/usr/bin/env python3
import rclpy
import math
import transforms3d

from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy import qos


class SiloBroadcaster(Node):
    def __init__(self):
        super().__init__("silo_broadcaster_node")
        self.sent_timer = self.create_timer(0.05, self.timer_callback)

        self.tf_broadcaster1 = TransformBroadcaster(self)
        self.tf_broadcaster2 = TransformBroadcaster(self)
        self.tf_broadcaster3 = TransformBroadcaster(self)
        self.tf_broadcaster4 = TransformBroadcaster(self)
        self.tf_broadcaster5 = TransformBroadcaster(self)

        self.child_frame = ["silo1", "silo2", "silo3", "silo4", "silo5"]
        self.positions = [
            [8.14, 5.43],
            [8.89, 5.43],
            [9.64, 5.43],
            [10.39, 5.43],
            [11.14, 5.43],
        ]

        self.broadcaster_arr = [
            self.tf_broadcaster1,
            self.tf_broadcaster2,
            self.tf_broadcaster3,
            self.tf_broadcaster4,
            self.tf_broadcaster5,
        ]

    def timer_callback(self):
        for i in range(len(self.broadcaster_arr)):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "map"
            t.child_frame_id = self.child_frame[i]
            t.transform.translation.x = self.positions[i][0]
            t.transform.translation.y = self.positions[i][1]
            t.transform.translation.z = 0.0
            q = transforms3d.euler.euler2quat(0, 0, 0)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.broadcaster_arr[i].sendTransform(t)


def main():
    rclpy.init()

    sub = SiloBroadcaster()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
