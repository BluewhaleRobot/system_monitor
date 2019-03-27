#!/usr/bin/env python
# encoding=utf-8
# The MIT License (MIT)
#
# Copyright (c) 2018 Bluewhale Robot
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# Author: Randoms
#

import struct
import threading

import numpy as np
import rospy
import tf
from galileo_serial_server.msg import GalileoStatus
from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav_msgs.msg import Odometry

from config import BROADCAST_PORT, ROS_PACKAGE_PATH, TF_ROT, TF_TRANS


class UDPStatusService(threading.Thread):

    def __init__(self, monitor_server, galileo_status, galileo_status_lock):
        super(UDPStatusService, self).__init__()
        # stop flag
        self._stop = threading.Event()
        self._stop.set()
        self.listener = tf.TransformListener(True, rospy.Duration(10.0))
        self.send_data = bytearray(
            [205, 235, 215, 36, 0x00, 0x00, 0x00, 0x00,
             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
             0x00, 0x00, 0x00, 0x00,
             ])
        self.monitor_server = monitor_server
        self.robot_pose_stamped = None
        self.galileo_status = galileo_status
        self.galileo_status_lock = galileo_status_lock
        self.odom_flag = False
        self.image_flag = False

        def update_odom(odom):
            self.odom_flag = True
            self.robot_pose_stamped = PoseStamped()
            self.robot_pose_stamped.pose = odom.pose.pose  # 更新坐标
            self.robot_pose_stamped.header = odom.header  # 更新坐标

        def update_camera_status(status):
            self.image_flag = True

        self.odom_sub = rospy.Subscriber("/bWmono/Odom", Odometry, update_odom)
        self.camera_sub = rospy.Subscriber(
            "/camera_node/camera_info", rospy.AnyMsg, update_camera_status)

    def stop(self):
        self.odom_sub.unregister()
        self.camera_sub.unregister()
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()

    def run(self):
        rate = rospy.Rate(30)
        self._stop.clear()
        t_ac = np.array([0., 0., 0.])
        theta_send = 0.
        while not self.stopped() and not rospy.is_shutdown():
            if self.monitor_server.get_connection_status():
                if self.odom_flag and self.robot_pose_stamped is not None:
                    # 将map坐标系转换成ORB_SLAM/World坐标系
                    current_pose = self.robot_pose_stamped.pose
                    t_bc = np.array(
                        [current_pose.position.x, current_pose.position.y, current_pose.position.z])
                    q = [current_pose.orientation.x, current_pose.orientation.y,
                         current_pose.orientation.z, current_pose.orientation.w]
                    m = tf.transformations.quaternion_matrix(q)
                    r_bc = m[:3, :3]

                    _, _, theta_send = tf.transformations.euler_from_matrix(
                        r_bc)
                    # 为了简化计算，下文的计算中 base_link 和 base_footprint 被看成是相同的坐标系

                    r_ab = TF_ROT.T
                    t_ab = -r_ab.dot(TF_TRANS)
                    t_ac = r_ab.dot(t_bc) + t_ab

                self.send_data[4:8] = map(ord, struct.pack('f', t_ac[0]))
                self.send_data[8:12] = map(ord, struct.pack('f', t_ac[1]))
                self.send_data[12:16] = map(ord, struct.pack('f', t_ac[2]))
                with self.galileo_status_lock:
                    self.send_data[16:20] = map(
                        ord, struct.pack('f', self.galileo_status.power))
                    self.send_data[24:28] = map(
                        ord, struct.pack('f', theta_send))
                    self.send_data[28:32] = map(ord, struct.pack(
                        'i', self.galileo_status.targetStatus))
                    self.send_data[32:36] = map(ord, struct.pack(
                        'i', self.galileo_status.targetNumID))
                    self.send_data[40:44] = map(ord, struct.pack(
                        'i', self.galileo_status.loopStatus))
                    self.send_data[36:40] = map(ord, struct.pack(
                        'i', self.galileo_status.chargeStatus))

                    if self.galileo_status.navStatus == 1:
                        statu0 = 0x01  # 导航状态
                    else:
                        statu0 = 0x00
                    if self.image_flag:
                        statu1 = 0x02  # 视觉摄像头
                    else:
                        statu1 = 0x00
                    if self.galileo_status.visualStatus != -1:
                        statu2 = 0x04  # 视觉系统状态
                    else:
                        statu2 = 0x00
                    if self.galileo_status.visualStatus == 1:
                        statu3 = 0x08  # 视觉系统状态
                    else:
                        statu3 = 0x00
                    if self.galileo_status.gcStatus == 1:
                        status4 = 0x10  # ORB_SLAM 内存回收状态
                    else:
                        status4 = 0
                    if self.galileo_status.gbaStatus == 1:
                        status5 = 0x20  # ORB_SLAM2 GBA状态，一般对应LoopClosing
                    else:
                        status5 = 0x00
                self.send_data[20] = statu0 + statu1 + \
                    statu2 + statu3 + status4 + status5
            self.send_data[3] = len(self.send_data) - 4
            self.monitor_server.sendto(bytes(self.send_data))
            rate.sleep()
