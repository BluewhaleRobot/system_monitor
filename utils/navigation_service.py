#!/usr/bin/env python3
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
# Author: Randoms, Xie fusheng
#

import os
import subprocess
import threading
import time

import psutil
import rospy

from .config import ROS_PACKAGE_PATH


class NavigationService(threading.Thread):
    # orb_slam建图线程
    def __init__(self, galileo_status, galileo_status_lock):
        super(NavigationService, self).__init__()
        self._stop = threading.Event()
        self._stop.set()
        self.p = None
        self.ps_process = None
        self.speed = 1
        self.galileo_status = galileo_status
        self.galileo_status_lock = galileo_status_lock

    def stop(self):
        if self.p != None:
            try:
                self.ps_process = psutil.Process(pid=self.p.pid)
                for child in self.ps_process.children(recursive=True):
                    child.kill()
                self.ps_process.kill()
            except Exception:
                pass
        self.p = None
        self._stop.set()
        self.__init__(self.galileo_status, self.galileo_status_lock)

    def stopped(self):
        return self._stop.isSet()

    def setspeed(self, speed):
        self.speed = speed

    def run(self):
        self._stop.clear()
        if self.speed == 1:
            cmd = "roslaunch nav_test tank_blank_map1.launch"
        elif self.speed == 2:
            cmd = "roslaunch nav_test tank_blank_map2.launch"
        elif self.speed == 3:
            cmd = "roslaunch nav_test tank_blank_map3.launch"
        elif self.speed == 0:
            cmd = "roslaunch nav_test tank_blank_map0.launch"

        new_env = os.environ.copy()
        new_env['ROS_PACKAGE_PATH'] = ROS_PACKAGE_PATH
        while not self.stopped() and not rospy.is_shutdown():
            if self.p == None and not self.stopped():
                self.p = subprocess.Popen(cmd, shell=True, env=new_env)
                self.ps_process = psutil.Process(pid=self.p.pid)
            else:
                if not self.ps_process.is_running():
                    break
            time.sleep(0.1)
        self.stop()
