#!/usr/bin/env python3
# Copyright 2011 Brown University Robotics.
# Copyright 2017 Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import sys

import geometry_msgs.msg
import std_msgs.msg
import rclpy
from rclpy import qos
from rclpy.qos import QoSProfile
from linkattacher_msgs.srv import AttachLink, DetachLink

if sys.platform == "win32":
    import msvcrt
else:
    import termios
    import tty


msg = """
This node takes keypresses from the keyboard and publishes them
as Twist messages. It works best with a US keyboard layout.
---------------------------
Moving around:
        i     
   j    k    l
        ,     

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

u : left arm
o : right arm
m : left hand
. : right hand
t : direction left arm
b : direction right arm


anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    "i": (1, 0, 0, 0),
    # "o": (1, 0, 0, -1),
    "j": (0, 0, 0, 1),
    "l": (0, 0, 0, -1),
    # "u": (1, 0, 0, 1),
    ",": (-1, 0, 0, 0),
    # ".": (-1, 0, 0, 1),
    # "m": (-1, 0, 0, -1),
    "O": (1, -1, 0, 0),
    "I": (1, 0, 0, 0),
    "J": (0, 1, 0, 0),
    "L": (0, -1, 0, 0),
    "U": (1, 1, 0, 0),
    "<": (-1, 0, 0, 0),
    ">": (-1, -1, 0, 0),
    "M": (-1, 1, 0, 0),
    # "t": (0, 0, 1, 0),
    # "b": (0, 0, -1, 0),
}

# speedBindings = {
#     "q": (1.1, 1.1),
#     "z": (0.9, 0.9),
#     "w": (1.1, 1),
#     "x": (0.9, 1),
#     "e": (1, 1.1),
#     "c": (1, 0.9),
# }

gripperBindings = {
    "u": 1,
    "o": 2,
    "m": 3,
    ".": 4,
    "t": 5,
    "b": 6,
}


def getKey(settings):
    if sys.platform == "win32":
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == "win32":
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == "win32":
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node("teleop_twist_keyboard")
    pub = node.create_publisher(
        geometry_msgs.msg.Twist, "cmd_vel", qos_profile=qos.qos_profile_system_default
    )
    pub_left_arm = node.create_publisher(
        std_msgs.msg.Float32MultiArray,
        "target_pose_left_arm",
        qos_profile=qos.qos_profile_system_default,
    )
    pub_right_arm = node.create_publisher(
        std_msgs.msg.Float32MultiArray,
        "target_pose_right_arm",
        qos_profile=qos.qos_profile_system_default,
    )
    pub_left_hand = node.create_publisher(
        std_msgs.msg.Bool,
        "target_left_hand",
        qos_profile=qos.qos_profile_system_default,
    )
    pub_right_hand = node.create_publisher(
        std_msgs.msg.Bool,
        "target_right_hand",
        qos_profile=qos.qos_profile_system_default,
    )

    client_attach = node.create_client(AttachLink, "/ATTACHLINK")
    client_detach = node.create_client(DetachLink, "/DETACHLINK")

    request_attach = AttachLink.Request()
    request_attach.model1_name = "abu_bot"
    request_attach.link1_name = "left_1_gripper_hand_link"
    request_attach.model2_name = "seed_blue_10"
    request_attach.link2_name = "link_1"
    request_detach = DetachLink.Request()
    request_detach.model1_name = "abu_bot"
    request_detach.link1_name = "left_1_gripper_hand_link"
    request_detach.model2_name = "seed_blue_10"
    request_detach.link2_name = "link_1"

    speed = 0.5
    turn = 1.0
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0.0
    dir_left = 0.0
    dir_right = 0.0
    left_arm = [0.0, 0.0, 0.0]
    right_arm = [0.0, 0.0, 0.0]
    left_hand = False
    right_hand = False
    state_left_arm = 0
    state_right_arm = 0
    state_left_hand = 0
    state_right_hand = 0
    state_dir_left_arm = 0
    state_dir_right_arm = 0

    try:
        print(msg)
        print(vels(speed, turn))
        while True:
            key = getKey(settings)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in gripperBindings.keys():
                if gripperBindings[key] == 1:
                    if state_left_arm == 0:
                        left_arm = [dir_left, 1.57, 1.57]
                        state_left_arm = 1
                    elif state_left_arm == 1:
                        left_arm = [dir_left, -1.256, 1.57]
                        # left_arm = [dir_left, 0.0, 1.57]
                        state_left_arm = 0
                elif gripperBindings[key] == 2:
                    if state_right_arm == 0:
                        right_arm = [dir_right, 1.57, 1.57]
                        state_right_arm = 1
                    elif state_right_arm == 1:
                        right_arm = [dir_right, -1.256, 1.57]
                        state_right_arm = 0
                elif gripperBindings[key] == 3:
                    if state_left_hand == 0:
                        left_hand = True
                        # client_attach.call_async(request_attach)
                        state_left_hand = 1
                    elif state_left_hand == 1:
                        left_hand = False
                        # client_detach.call_async(request_detach)
                        state_left_hand = 0
                elif gripperBindings[key] == 4:
                    if state_right_hand == 0:
                        right_hand = True
                        state_right_hand = 1
                    elif state_right_hand == 1:
                        right_hand = False
                        state_right_hand = 0
                elif gripperBindings[key] == 5:
                    if state_dir_left_arm == 0:
                        dir_left = 3.14
                        state_dir_left_arm = 1
                    elif state_dir_left_arm == 1:
                        dir_left = 0.0
                        state_dir_left_arm = 0
                    left_arm[0] = dir_left
                elif gripperBindings[key] == 6:
                    if state_dir_right_arm == 0:
                        dir_right = -3.14
                        state_dir_right_arm = 1
                    elif state_dir_right_arm == 1:
                        dir_right = 0.0
                        state_dir_right_arm = 0
                    right_arm[0] = dir_right
            # elif key in speedBindings.keys():
            #     speed = speed * speedBindings[key][0]
            #     turn = turn * speedBindings[key][1]

            #     print(vels(speed, turn))
            #     if status == 14:
            #         print(msg)
            #     status = (status + 1) % 15
            else:
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0
                if key == "\x03":
                    break

            joint_left_arm = std_msgs.msg.Float32MultiArray()
            joint_right_arm = std_msgs.msg.Float32MultiArray()
            joint_left_hand = std_msgs.msg.Bool()
            joint_right_hand = std_msgs.msg.Bool()
            joint_left_arm.data = left_arm
            joint_right_arm.data = right_arm
            joint_left_hand.data = left_hand
            joint_right_hand.data = right_hand
            twist = geometry_msgs.msg.Twist()
            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = th * turn
            pub.publish(twist)
            pub_left_arm.publish(joint_left_arm)
            pub_left_hand.publish(joint_left_hand)
            pub_right_arm.publish(joint_right_arm)
            pub_right_hand.publish(joint_right_hand)

    except Exception as e:
        print(e)

    finally:
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

        restoreTerminalSettings(settings)


if __name__ == "__main__":
    main()
