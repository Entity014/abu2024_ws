#!/usr/bin/env python3
import rclpy
import tf2_ros
import tf2_geometry_msgs

from rclpy import qos
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped, Vector3


class SiloListener(Node):
    def __init__(self):
        super().__init__("silo_listener_node")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.sub_silo = self.create_subscription(
            String,
            "silo/type",
            self.sub_silo_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_silo

        self.pub_silo_pos = self.create_publisher(
            Vector3, "silo/pos", qos_profile=qos.qos_profile_system_default
        )
        self.silo_type = "silo1"

    def sub_silo_callback(self, msgin):
        self.silo_type = msgin.data
        try:
            msg = Vector3()
            transform = self.tf_buffer.lookup_transform(
                self.silo_type, "map", rclpy.time.Time()
            )
            position = transform.transform.translation
            # self.get_logger().info(
            #     "Position: x={}, y={}, z={}".format(-position.x, position.y, position.z)
            # )
            msg.x = -position.x
            msg.y = position.y
            msg.z = position.z
            self.pub_silo_pos.publish(msg)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            self.get_logger().warning("Failed to get TF")


def main():
    rclpy.init()

    sub = SiloListener()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
