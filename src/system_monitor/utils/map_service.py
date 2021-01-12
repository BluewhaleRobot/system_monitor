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
# Author: Randoms, Xiefusheng
#

import os
import subprocess
import threading
import time

import psutil
import rospy

from .config import ROS_PACKAGE_PATH
from .scale_orb import ScaleORB
from pymongo import MongoClient
from actionlib.simple_action_client import SimpleActionClient
from ORB_SLAM2.msg import LoadMapAction, LoadMapActionGoal


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
        os.system("pkill -f 'roslaunch startup map.launch'")
        os.system("pkill -f 'roslaunch ORB_SLAM2 map_fake.launch'")
        os.system("pkill -f 'roslaunch startup update.launch'")
        os.system("pkill -f 'roslaunch ORB_SLAM2 update_fake.launch'")
        self.P = None
        self._stop.set()
        self.__init__(self.galileo_status, self.galileo_status_lock)

    def stopped(self):
        return self._stop.isSet()

    def run(self):
        self._stop.clear()
        # 多摄像头参数，0表示使用前摄像头建图，1表示使用后摄像头建图
        camera_id = int(rospy.get_param("~camera_id", -1))
        calib_flag = rospy.get_param("~calib_flag", False)
        cmd = "roslaunch startup map.launch"
        if camera_id == 0:
            cmd = "roslaunch startup map.launch camera_type:=front"
        if camera_id == 1:
            cmd = "roslaunch startup map.launch camera_type:=back"
        if self.fake_flag:
            cmd = "roslaunch ORB_SLAM2 map_fake.launch"
        if self.update:
            cmd = "roslaunch startup update.launch"
            if self.fake_flag:
                cmd = "roslaunch ORB_SLAM2 update_fake.launch"
        if calib_flag:
            cmd += " calib_flag:=true"
        new_env = os.environ.copy()
        new_env['ROS_PACKAGE_PATH'] = ROS_PACKAGE_PATH
        while not self.stopped() and not rospy.is_shutdown():
            if self.P == None and not self.stopped():
                self.P = subprocess.Popen(cmd, shell=True, env=new_env)
                self.ps_process = psutil.Process(pid=self.P.pid)
                if self.update:
                    # 同时载入更新地图
                    c = MongoClient()
                    db = c["bwbot_galileo_debug"]["config"]
                    config_data = db.find_one({})
                    map_name = config_data["default_map"]
                    if map_name == "":
                        continue
                    client = SimpleActionClient("/ORB_SLAM2/map_load", LoadMapAction)
                    client.wait_for_server()
                    goal = LoadMapActionGoal()
                    goal.goal.map_name = map_name
                    client.send_goal(goal.goal)
                    load_state = client.wait_for_result()
                    if not load_state:
                        break
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
