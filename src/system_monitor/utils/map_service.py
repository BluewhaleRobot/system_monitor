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

import os
import subprocess
import threading
import time

import psutil
import rospy

from config import ROS_PACKAGE_PATH
from scale_orb import ScaleORB


class MapService(threading.Thread):
    # orb_slam建图线程
    def __init__(self, galileo_status, galileo_status_lock, update=False):
        super(MapService, self).__init__()
        self._stop = threading.Event()
        self._stop.set()
        self.P = None
        self.ps_process = None
        self.scale_orb_thread = None
        self.update = update
        self.galileo_status = galileo_status
        self.galileo_status_lock = galileo_status_lock
        self.fake_flag = rospy.get_param("~fake", False)

    def stop(self):
        if self.scale_orb_thread != None:
            self.scale_orb_thread.stop()
            self.scale_orb_thread = None

        if self.P != None:
            self.ps_process = psutil.Process(pid=self.P.pid)
            for child in self.ps_process.children(recursive=True):
                child.kill()
            self.ps_process.kill()
        self.P = None
        self._stop.set()
        self.__init__(self.galileo_status, self.galileo_status_lock)

    def stopped(self):
        return self._stop.isSet()

    def run(self):
        self._stop.clear()
        # 多摄像头参数，0表示使用前摄像头建图，1表示使用后摄像头建图
        camera_id = int(rospy.get_param("~camera_id", -1))
        cmd = "roslaunch ORB_SLAM2 map.launch"
        if camera_id == 0:
            cmd = "roslaunch ORB_SLAM2 map_front.launch"
        if camera_id == 1:
            cmd = "roslaunch ORB_SLAM2 map_back.launch"
        if self.fake_flag:
            cmd = "roslaunch ORB_SLAM2 map_fake.launch"
        if self.update:
            cmd = "roslaunch nav_test update_map.launch"
            if self.fake_flag:
                cmd = "roslaunch nav_test update_map_fake.launch"
        new_env = os.environ.copy()
        new_env['ROS_PACKAGE_PATH'] = ROS_PACKAGE_PATH
        while not self.stopped() and not rospy.is_shutdown():
            if self.P == None and not self.stopped():
                self.P = subprocess.Popen(cmd, shell=True, env=new_env)
                self.ps_process = psutil.Process(pid=self.P.pid)
            else:
                if self.ps_process.is_running():
                    if self.scale_orb_thread == None:
                        self.scale_orb_thread = ScaleORB(
                            self.galileo_status, self.galileo_status_lock)
                        self.scale_orb_thread.start()
                else:
                    if self.scale_orb_thread != None:
                        self.scale_orb_thread.stop()
                        self.scale_orb_thread = None
                    break
            time.sleep(0.1)
        self.stop()
