#!/usr/bin/env python3

import rclpy
import numpy as np
import cv2

from rclpy.node import Node
from rclpy import qos, Parameter
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Vector3, Quaternion, Point
from tf2_ros import TransformBroadcaster, TransformStamped


class SeedDetection(Node):
    def __init__(self):
        super().__init__("seed_detection_node")
        self.sub_info = self.create_subscription(
            CameraInfo,
            "camera/color/camera_info",
            self.sub_info_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_info

        self.sub_img = self.create_subscription(
            Image,
            "camera/color/image_raw",
            self.sub_image_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_img

        self.sub_depth = self.create_subscription(
            Image,
            "camera/depth/image_rect_raw",
            self.sub_depth_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_depth

        self.sub_odom = self.create_subscription(
            Odometry,
            "odom",
            self.sub_odom_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.pub_marker = self.create_publisher(
            Marker, "marker_topic", qos_profile=qos.qos_profile_system_default
        )
        # self.timer = self.create_timer(0.5, self.timer_callback)

        self.broadcaster = TransformBroadcaster(self)
        self.br = CvBridge()
        self.image = np.zeros((480, 640, 3))
        self.depth = np.zeros((480, 640))
        self.distance = 0.0
        self.x = 0.0

    def sub_image_callback(self, image):
        self.image = self.br.imgmsg_to_cv2(image, "bgr8")
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([90, 0, 0])
        upper_blue = np.array([130, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        blue_detected = cv2.bitwise_and(self.image, self.image, mask=mask)
        gray = cv2.cvtColor(blue_detected, cv2.COLOR_BGR2GRAY)
        contours, _ = cv2.findContours(
            gray.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        i = 0

        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            epsilon = 0.04 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            area = cv2.contourArea(contour)
            M = cv2.moments(contour)

            if M["m00"] != 0 and area >= 400:
                i += 1
                centroid_x = int(M["m10"] / M["m00"])
                centroid_y = int(M["m01"] / M["m00"])
                self.x = np.interp(x, [0, 640], [-0.65, 0.65])
                self.distance = self.depth[centroid_y, centroid_x]
                cv2.drawContours(self.image, [approx], -1, (0, 255, 0), 2)
                cv2.circle(self.image, (centroid_x, centroid_y), 5, (0, 0, 255), -1)
                cv2.putText(
                    self.image,
                    f"{i}",
                    (x, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (255, 0, 0),
                    2,
                    cv2.LINE_AA,
                )

        cv2.imshow("camera", self.image)

        cv2.waitKey(1)

    def sub_depth_callback(self, image):
        self.depth = self.br.imgmsg_to_cv2(image, "32FC1")

    def sub_info_callback(self, info):
        pass

    def sub_odom_callback(self, odom):
        if self.distance != 0:
            marker_msg = Marker()
            transform = TransformStamped()
            marker_msg.header.frame_id = "odom"
            transform.header.frame_id = "odom"
            transform.child_frame_id = "basic_shapes"
            marker_msg.header.stamp = self.get_clock().now().to_msg()
            transform.header.stamp = self.get_clock().now().to_msg()
            marker_msg.ns = "basic_shapes"
            marker_msg.id = 0
            marker_msg.action = Marker.ADD
            marker_msg.type = Marker.CYLINDER
            marker_msg.pose.position = Point(
                x=odom.pose.pose.position.x - self.distance - 0.3,
                y=odom.pose.pose.position.y - self.x,
                z=0.05,
            )
            transform.transform.translation = Vector3(
                x=odom.pose.pose.position.x - self.distance - 0.3,
                y=odom.pose.pose.position.y - self.x,
                z=0.05,
            )
            marker_msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=0.0)
            marker_msg.scale = Vector3(x=0.08, y=0.08, z=0.4)
            marker_msg.color.a = 1.0
            marker_msg.color.r = 1.0
            marker_msg.color.g = 0.0
            marker_msg.color.b = 0.0
            marker_msg.lifetime.sec = 1
            self.pub_marker.publish(marker_msg)
            self.broadcaster.sendTransform(transform)

        # print(odom.pose.pose.position)


def main(args=None):
    rclpy.init(args=args)

    sub = SeedDetection()
    rclpy.spin(sub)
    sub.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
