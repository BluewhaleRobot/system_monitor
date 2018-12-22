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
# Author: Randoms, Xiefusheng
#

import math
import threading
import time

import rospy
from geometry_msgs.msg import Pose, Pose2D


class ScaleORB(threading.Thread):

    def __init__(self, galileo_status, galileo_status_lock):
        super(ScaleORB, self).__init__()
        self._stop = threading.Event()
        self.car_odoms = 0.0
        self.camera_odoms = 0.0
        self.car_last_pose = None
        self.camera_last_pose = None
        self.scale = 1.
        self.car_pose_lock = threading.Lock()
        self.camera_pose_lock = threading.Lock()
        self.scale_lock = threading.Lock()
        self.car_begin_flag = True
        self.camera_begin_flag = True
        self.galileo_status = galileo_status
        self.galileo_status_lock = galileo_status_lock

    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()

    def run(self):

        def update_car(pose2d):
            self.car_pose_lock.acquire()
            self.scale_lock.acquire()
            if self.car_begin_flag:
                self.car_last_pose = pose2d
                self.car_begin_flag = False
            self.car_odoms += math.sqrt((pose2d.x - self.car_last_pose.x) * (pose2d.x - self.car_last_pose.x) + (
                pose2d.y - self.car_last_pose.y) * (pose2d.y - self.car_last_pose.y))
            self.scale_lock.release()
            self.car_last_pose = pose2d
            self.car_pose_lock.release()

        def update_camera(pose):
            self.camera_pose_lock.acquire()
            self.scale_lock.acquire()
            if self.camera_begin_flag:
                self.camera_last_pose = pose
                self.camera_begin_flag = False
            self.camera_odoms += math.sqrt((pose.position.x - self.camera_last_pose.position.x) * (pose.position.x - self.camera_last_pose.position.x) + (
                pose.position.z - self.camera_last_pose.position.z) * (pose.position.z - self.camera_last_pose.position.z))
            self.scale_lock.release()
            self.camera_last_pose = pose
            self.camera_pose_lock.release()

        while not rospy.is_shutdown():
            with self.galileo_status_lock:
                if self.galileo_status.visualStatus == 1:
                    break
            time.sleep(0.5)
        if rospy.is_shutdown():
            self.stop()
            return
        car_sub = rospy.Subscriber(
            "/xqserial_server/Pose2D", Pose2D, update_car)
        camera_sub = rospy.Subscriber("/ORB_SLAM/Camera", Pose, update_camera)
        while not self.stopped() and not rospy.is_shutdown():
            time.sleep(1)
        car_sub.unregister()
        camera_sub.unregister()
        self.stop()

    def save_scale(self):
        self.scale_lock.acquire()
        if self.camera_odoms > 0.01:
            self.scale = self.car_odoms / self.camera_odoms
        else:
            self.scale = 1.0
        with open("/home/xiaoqiang/slamdb/scale.txt", 'a+') as fp3:
            fp3.write(str(self.scale))
            fp3.write('\n')
        self.scale_lock.release()
