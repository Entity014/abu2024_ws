#!/usr/bin/env python3
import os
import rclpy
import cv2
import numpy as np
from ultralytics import YOLO

from rclpy.node import Node
from std_msgs.msg import String, Int8, Int8MultiArray
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
from rclpy import qos


class SiloDetection(Node):
    def __init__(self):
        super().__init__("silo_detection_node")
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
        self.sub_depth_image = self.create_subscription(
            Image,
            "camera/color/image_raw",
            self.sub_depth_callback,
            10,
        )
        self.sub_depth_image

        self.pub_silo_array = self.create_publisher(
            Int8MultiArray, "silo/array", qos_profile=qos.qos_profile_system_default
        )
        self.pub_silo = self.create_publisher(
            Image, "silo/image", qos_profile=qos.qos_profile_system_default
        )
        self.sent_timer = self.create_timer(0.05, self.timer_callback)

        self.frame_silo = np.zeros((480, 640, 3), dtype=np.uint8)
        self.bridge = CvBridge()
        self.team = "none"
        self.team_arr = ["blue", "purple", "red"]
        self.robot_main_state = 0

        self.silo_arr = np.zeros((5, 3), dtype=np.int8)
        self.ball_arr = [
            [None, None, None],
            [None, None, None],
            [None, None, None],
            [None, None, None],
            [None, None, None],
        ]

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

    def sub_team_callback(self, msgin):
        self.team = msgin.data

    def sub_robot_main_callback(self, msgin):
        self.robot_main_state = msgin.data

    def sub_depth_callback(self, msgin):
        self.frame_silo = cv2.resize(
            self.bridge.imgmsg_to_cv2(msgin, desired_encoding="bgr8"), (640, 480)
        )

    def timer_callback(self):
        if self.team != "none" and self.robot_main_state == 11:
            msg_silo = Int8MultiArray()
            self.frame_silo, _ = self.predict_and_detect(
                self.model, self.frame_silo, classes=[], conf=0.5
            )
            for index, value in enumerate(self.ball_arr):
                for index2, value2 in enumerate(value):
                    if value2 is not None:
                        if self.team_arr[int(value2.cls[0])] == self.team.lower():
                            self.silo_arr[index][index2] = 1
                        else:
                            self.silo_arr[index][index2] = 2
                    else:
                        self.silo_arr[index][index2] = 0

            msg_silo.data = self.silo_arr.flatten().tolist()
            self.pub_silo_array.publish(msg_silo)

            cv2.line(self.frame_silo, (100, 0), (100, 480), (255, 0, 0), 2)
            cv2.line(self.frame_silo, (260, 0), (260, 480), (255, 0, 0), 2)
            cv2.line(self.frame_silo, (410, 0), (410, 480), (255, 0, 0), 2)
            cv2.line(self.frame_silo, (550, 0), (550, 480), (255, 0, 0), 2)
            cv2.line(self.frame_silo, (0, 230), (640, 230), (255, 0, 0), 2)
            cv2.line(self.frame_silo, (0, 190), (640, 190), (255, 0, 0), 2)
        self.pub_silo.publish(
            self.bridge.cv2_to_imgmsg(self.frame_silo, encoding="bgr8")
        )
        cv2.imshow("Camera2", self.frame_silo)
        cv2.waitKey(1)

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
            self.ball_arr = [
                [None, None, None],
                [None, None, None],
                [None, None, None],
                [None, None, None],
                [None, None, None],
            ]
            for box in result.boxes:
                if result.names[int(box.cls[0])] == "red":
                    color = (0, 0, 255)
                elif result.names[int(box.cls[0])] == "blue":
                    color = (255, 0, 0)
                else:
                    color = (145, 8, 160)

                if box.xywh[0][0] <= 100:
                    if box.xywh[0][1] >= 250:
                        self.ball_arr[0][0] = box
                    elif box.xywh[0][1] < 230 and box.xywh[0][1] >= 190:
                        self.ball_arr[0][1] = box
                    elif box.xywh[0][1] < 190:
                        self.ball_arr[0][2] = box
                elif box.xywh[0][0] > 100 and box.xywh[0][0] <= 260:
                    if box.xywh[0][1] >= 230:
                        self.ball_arr[1][0] = box
                    elif box.xywh[0][1] < 230 and box.xywh[0][1] >= 190:
                        self.ball_arr[1][1] = box
                    elif box.xywh[0][1] < 190:
                        self.ball_arr[1][2] = box
                elif box.xywh[0][0] > 260 and box.xywh[0][0] <= 410:
                    if box.xywh[0][1] >= 230:
                        self.ball_arr[2][0] = box
                    elif box.xywh[0][1] < 230 and box.xywh[0][1] >= 190:
                        self.ball_arr[2][1] = box
                    elif box.xywh[0][1] < 190:
                        self.ball_arr[2][2] = box
                elif box.xywh[0][0] > 410 and box.xywh[0][0] <= 550:
                    if box.xywh[0][1] >= 230:
                        self.ball_arr[3][0] = box
                    elif box.xywh[0][1] < 230 and box.xywh[0][1] >= 190:
                        self.ball_arr[3][1] = box
                    elif box.xywh[0][1] < 190:
                        self.ball_arr[3][2] = box
                elif box.xywh[0][0] > 550:
                    if box.xywh[0][1] >= 230:
                        self.ball_arr[4][0] = box
                    elif box.xywh[0][1] < 230 and box.xywh[0][1] >= 190:
                        self.ball_arr[4][1] = box
                    elif box.xywh[0][1] < 190:
                        self.ball_arr[4][2] = box

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

    sub = SiloDetection()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
