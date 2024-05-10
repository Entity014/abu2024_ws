#!/usr/bin/env python3
import rclpy
import math
import cv2
import numpy as np
import datetime

from rclpy.node import Node
from std_msgs.msg import Int8, String, Bool
from sensor_msgs.msg import LaserScan
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
        self.sub_lidar = self.create_subscription(
            LaserScan,
            "scan",
            self.sub_lidar_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )

        self.pub_color = self.create_publisher(
            Bool, "color/found", qos_profile=qos.qos_profile_system_default
        )
        self.pub_move = self.create_publisher(
            String, "color/move", qos_profile=qos.qos_profile_system_default
        )
        self.sent_timer = self.create_timer(0.05, self.timer_callback)

        self.cap = cv2.VideoCapture("/dev/video4")
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")  # Codec for MP4 format
        self.out = cv2.VideoWriter(
            f"{datetime.datetime.now()}.mp4", fourcc, 20.0, (640, 480)
        )  # 'output.mp4' is the output file name
        self.frame = np.zeros((480, 640, 3), dtype=np.uint8)
        self.r_low = np.array([160, 110, 0])
        self.r_up = np.array([179, 255, 255])
        self.b_low = np.array([80, 100, 180])
        self.b_up = np.array([125, 255, 255])
        self.team = "none"
        self.cx, self.cy = (0, 0)

        self.start_time = None
        self.state = 0
        self.detect_state = 0
        self.found = False

    def sub_team_callback(self, msgin):
        self.team = msgin.data

    def sub_lidar_callback(self, msgin):
        if msgin.ranges[1799] < 0.4:
            self.detect_state = 1
        else:
            self.detect_state = 0

    def timer_callback(self):
        ret, self.frame = self.cap.read()
        self.frame = cv2.flip(self.frame, 0)
        self.frame = cv2.flip(self.frame, 1)
        blur_image = cv2.GaussianBlur(self.frame, (15, 15), 0)
        self.hsv_frame = cv2.cvtColor(blur_image, cv2.COLOR_BGR2HSV)
        if self.team == "BLUE":
            mask = cv2.inRange(self.hsv_frame, self.b_low, self.b_up)
        elif self.team == "RED":
            mask = cv2.inRange(self.hsv_frame, self.r_low, self.r_up)
        if self.team != "none":
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            self.search_contours(contours)
            if self.detect_state == 1:
                msg_move = String()
                # B = self.frame[
                #     round(self.frame.shape[0] / 2), round(self.frame.shape[1] / 2)
                # ][0]
                # G = self.frame[
                #     round(self.frame.shape[0] / 2), round(self.frame.shape[1] / 2)
                # ][1]
                # R = self.frame[
                #     round(self.frame.shape[0] / 2), round(self.frame.shape[1] / 2)
                # ][2]
                if self.team == "BLUE":
                    # if B >= 240 and (abs(int(R) - int(G)) >= 30):
                    if self.found:
                        msg_move.data = "DONE"
                    else:
                        msg_move.data = "FAIL"
                elif self.team == "RED":
                    if self.found:
                        msg_move.data = "DONE"
                    else:
                        msg_move.data = "FAIL"
                self.pub_move.publish(msg_move)
            cv2.line(self.frame, (285, 0), (285, 480), (255, 0, 0), 2)
            cv2.line(self.frame, (355, 0), (355, 480), (255, 0, 0), 2)
        self.out.write(self.frame)
        cv2.imshow("Camera", self.frame)
        cv2.waitKey(1)

    def search_contours(self, contours):
        msg_move = String()
        msg_found = Bool()
        # for contour in contours:
        if len(contours) != 0:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            x, y, w, h = cv2.boundingRect(largest_contour)
            epsilon = 0.005 * cv2.arcLength(largest_contour, True)
            approx = cv2.approxPolyDP(largest_contour, epsilon, True)
            if len(approx) >= 10:
                self.found = True
                moments = cv2.moments(largest_contour)
                if moments["m00"] != 0:
                    self.cx = int(moments["m10"] / moments["m00"])
                    self.cy = int(moments["m01"] / moments["m00"])
                    if self.detect_state == 0:
                        if self.cx <= 285:
                            msg_move.data = "LEFT"
                            self.state = 0
                        elif self.cx >= 355:
                            msg_move.data = "RIGHT"
                            self.state = 0
                        else:
                            msg_move.data = "CENTER"
                            # if self.state == 0:
                            #     self.start_time = time.time()
                            #     self.state = 1
                            # elif self.state == 1:
                            #     elapsed_time = time.time() - self.start_time
                            #     if elapsed_time > 1:
                            #         msg_move.data = "CENTER"
                            #         self.detect_state = 1
                        self.pub_move.publish(msg_move)
                    cv2.circle(
                        self.frame, (self.cx, self.cy), 5, (0, 0, 255), -1
                    )  # Mark center with a red circle
                    cv2.putText(
                        self.frame,
                        f"{len(approx)}",
                        (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (255, 0, 255),
                        2,
                        cv2.LINE_AA,
                    )
                    cv2.drawContours(self.frame, [approx], -1, (0, 255, 0), 2)
        else:
            self.found = False
        msg_found.data = self.found
        self.pub_color.publish(msg_found)


def main():
    rclpy.init()

    sub = BallDetection()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
