#!/usr/bin/env python3
import os
import rclpy
import cv2
import numpy as np
from ultralytics import YOLO

from rclpy.node import Node
from std_msgs.msg import String, Bool, Int8
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
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
        self.sub_team = self.create_subscription(
            Int8,
            "robot/main",
            self.sub_robot_main_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_lidar = self.create_subscription(
            LaserScan,
            "scan",
            self.sub_lidar_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )

        self.pub_color = self.create_publisher(
            String, "color/type", qos_profile=qos.qos_profile_system_default
        )
        self.pub_move = self.create_publisher(
            String, "color/move", qos_profile=qos.qos_profile_system_default
        )
        self.pub_detect = self.create_publisher(
            Image, "color/detect", qos_profile=qos.qos_profile_system_default
        )
        self.sent_timer = self.create_timer(0.05, self.timer_callback)

        self.cap = cv2.VideoCapture("/dev/video2")
        self.frame = np.zeros((480, 640, 3), dtype=np.uint8)
        self.bridge = CvBridge()
        self.team = "none"
        self.robot_main_state = 0

        self.max_ball_team = None
        self.max_ball_purple = None
        self.detect_state = 0
        self.color_type = String()

        self.model = YOLO(
            os.path.join(
                os.path.expanduser("~"),
                "robot_ws",
                "install",
                "abu_core",
                "share",
                "abu_core",
                "weights",
                "bestn2.pt",
            )
        )

    def sub_robot_main_callback(self, msgin):
        self.robot_main_state = msgin.data

    def sub_team_callback(self, msgin):
        self.team = msgin.data

    def sub_lidar_callback(self, msgin):
        angles = np.linspace(msgin.angle_min, msgin.angle_max, len(msgin.ranges))
        theta = [
            (r if (theta < 3.15 and theta > 2.83) else np.inf)
            for r, theta in zip(msgin.ranges, angles)
        ]
        # self.get_logger().info(f"{theta[1760]}")
        if theta[1760] < 0.41 and (
            self.robot_main_state == 4 or self.robot_main_state == 5
        ):
            self.detect_state = 1
        else:
            self.detect_state = 0

    def timer_callback(self):
        ret, self.frame = self.cap.read()
        self.frame = cv2.flip(self.frame, 0)
        self.frame = cv2.flip(self.frame, 1)
        if self.team != "none" and (
            self.robot_main_state == 4 or self.robot_main_state == 5
        ):
            self.frame, _ = self.predict_and_detect(
                self.model, self.frame, classes=[], conf=0.5
            )
            if self.max_ball_team is not None:
                self.color_type.data = self.team
            # elif self.max_ball_purple is not None:
            #     self.color_type.data = "PURPLE"
            else:
                self.color_type.data = "NONE"
            if self.detect_state == 0:
                msg_move = String()
                if self.color_type.data == self.team:
                    if self.max_ball_team.xywh[0][0] <= 325:
                        msg_move.data = "LEFT"
                    elif self.max_ball_team.xywh[0][0] >= 395:
                        msg_move.data = "RIGHT"
                    else:
                        msg_move.data = "CENTER"
                # elif self.color_type.data == "PURPLE":
                #     if self.max_ball_purple.xywh[0][0] <= 325:
                #         msg_move.data = "LEFT"
                #     elif self.max_ball_purple.xywh[0][0] >= 395:
                #         msg_move.data = "RIGHT"
                #     else:
                #         msg_move.data = "CENTER"
                self.pub_move.publish(msg_move)
            elif self.detect_state == 1:
                msg_move = String()
                if self.color_type.data == self.team:
                    msg_move.data = "DONE"
                else:
                    msg_move.data = "FAIL"
                self.pub_move.publish(msg_move)
            self.pub_color.publish(self.color_type)

            cv2.line(self.frame, (325, 0), (325, 480), (255, 0, 0), 2)
            cv2.line(self.frame, (395, 0), (395, 480), (255, 0, 0), 2)
        self.pub_detect.publish(self.bridge.cv2_to_imgmsg(self.frame, encoding="bgr8"))
        # cv2.imshow("Camera", self.frame)
        # cv2.waitKey(1)

    def predict(self, chosen_model, img, classes=[], conf=0.5):
        if classes:
            results = chosen_model.predict(img, classes=classes, conf=conf)
        else:
            results = chosen_model.predict(img, conf=conf)

        return results

    def predict_and_detect(
        self,
        chosen_model,
        img,
        classes=[],
        conf=0.5,
        rectangle_thickness=2,
        text_thickness=1,
    ):
        color = (255, 0, 0)
        results = self.predict(chosen_model, img, classes, conf=conf)
        for result in results:
            max_area_team = 0
            max_area_purple = 0
            self.max_ball_team = None
            self.max_ball_purple = None
            for box in result.boxes:
                if result.names[int(box.cls[0])] == "red":
                    color = (0, 0, 255)
                elif result.names[int(box.cls[0])] == "blue":
                    color = (255, 0, 0)
                else:
                    color = (145, 8, 160)

                area = box.xywh[0][2] * box.xyxy[0][3]
                if (
                    self.team.lower() == result.names[int(box.cls[0])]
                    and area > max_area_team
                ):
                    max_area_team = area
                    self.max_ball_team = box
                    color = (0, 255, 0)
                if "purple" == result.names[int(box.cls[0])] and area > max_area_purple:
                    max_area_purple = area
                    self.max_ball_purple = box
                cv2.rectangle(
                    img,
                    (int(box.xyxy[0][0]), int(box.xyxy[0][1])),
                    (int(box.xyxy[0][2]), int(box.xyxy[0][3])),
                    color,
                    rectangle_thickness,
                )
                cv2.circle(
                    img,
                    (
                        int(box.xywh[0][0]),
                        int(box.xywh[0][1]),
                    ),
                    5,
                    (0, 0, 0),
                    -1,
                )
                cv2.putText(
                    img,
                    f"{result.names[int(box.cls[0])]}",
                    (int(box.xyxy[0][0]), int(box.xyxy[0][1]) - 10),
                    cv2.FONT_HERSHEY_PLAIN,
                    1,
                    color,
                    text_thickness,
                )
        return img, results


def main():
    rclpy.init()

    sub = BallDetection()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
