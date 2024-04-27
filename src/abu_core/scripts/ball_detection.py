#!/usr/bin/env python3
import rclpy
import math
import cv2
import numpy as np
import time

from rclpy.node import Node
from std_msgs.msg import Int8, String
from rclpy import qos


class BallDetection(Node):
    def __init__(self):
        super().__init__("ball_detection_node")
        self.sub_team = self.create_subscription(
            String,
            "robot/team",
            self.sub_team_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_team
        self.pub_color = self.create_publisher(
            String, "robot/color", qos_profile=qos.qos_profile_system_default
        )
        self.pub_move = self.create_publisher(
            String, "robot/move", qos_profile=qos.qos_profile_system_default
        )
        self.sent_timer = self.create_timer(0.05, self.timer_callback)

        self.cap = cv2.VideoCapture("/dev/video14")
        self.frame = np.zeros((480, 640, 3), dtype=np.uint8)
        self.r_low = np.array([120, 25, 25])
        self.r_up = np.array([175, 255, 175])
        self.b_low = np.array([85, 60, 45])
        self.b_up = np.array([120, 255, 255])
        self.team = "none"
        self.cx, self.cy = (0, 0)

        self.start_time = None
        self.state = 0
        self.detect_state = 0

    def sub_team_callback(self, msgin):
        self.team = msgin.data

    def timer_callback(self):
        ret, self.frame = self.cap.read()
        self.frame = cv2.flip(self.frame, 0)
        self.frame = cv2.flip(self.frame, 1)
        blur_image = cv2.GaussianBlur(self.frame, (15, 15), 0)
        hsv_frame = cv2.cvtColor(blur_image, cv2.COLOR_BGR2HSV)
        if self.team == "BLUE":
            mask = cv2.inRange(hsv_frame, self.b_low, self.b_up)
        elif self.team == "RED":
            mask = cv2.inRange(hsv_frame, self.r_low, self.r_up)
        if self.team != "none":
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            self.search_contours(contours)
            cv2.line(self.frame, (230, 0), (230, 480), (255, 0, 0), 2)
            cv2.line(self.frame, (280, 0), (280, 480), (255, 0, 0), 2)
        cv2.imshow("Camera", self.frame)
        cv2.waitKey(1)

    def search_contours(self, contours):
        msg_move = String()
        # for contour in contours:
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        x, y, w, h = cv2.boundingRect(largest_contour)
        epsilon = 0.001 * cv2.arcLength(largest_contour, True)
        approx = cv2.approxPolyDP(largest_contour, epsilon, True)
        if area > 3000 and len(approx) >= 10:
            moments = cv2.moments(largest_contour)
            self.cx = int(moments["m10"] / moments["m00"])
            self.cy = int(moments["m01"] / moments["m00"])
            if self.detect_state == 0:
                if self.cx <= 230:
                    msg_move.data = "LEFT"
                    self.state = 0
                elif self.cx >= 280:
                    msg_move.data = "RIGHT"
                    self.state = 0
                else:
                    if self.state == 0:
                        self.start_time = time.time()
                        self.state = 1
                    elif self.state == 1:
                        elapsed_time = time.time() - self.start_time
                        if elapsed_time > 1:
                            msg_move.data = "CENTER"
                            self.detect_state = 1
            if area > 230000 and self.detect_state == 1:
                msg_move.data = "DONE"
                self.detect_state = 0
            self.pub_move.publish(msg_move)
            # print(area)
            cv2.circle(
                self.frame, (self.cx, self.cy), 5, (0, 0, 255), -1
            )  # Mark center with a red circle
            cv2.putText(
                self.frame,
                f"{area}",
                (x, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255, 0, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.drawContours(self.frame, [approx], -1, (0, 255, 0), 2)


def main():
    rclpy.init()

    sub = BallDetection()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
