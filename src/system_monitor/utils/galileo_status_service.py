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
# Author: Randoms
#

import threading
import time

import rospy
from galileo_serial_server.msg import GalileoStatus
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64, Int32, Float32
from tf.transformations import euler_from_quaternion
import tf
import requests
import json
from .config import POWER_LOW


class GalileoStatusService(threading.Thread):

    def __init__(self, galileo_pub, monitor_server, galileo_status, galileo_status_lock):
        super(GalileoStatusService, self).__init__()
        # status
        self.galileo_pub = galileo_pub
        self.monitor_server = monitor_server
        self.visual_status = 0
        self.gc_status = 0
        self.gba_status = 0
        self.charge_status = 0
        self.power = 0
        self.control_speed_x = 0
        self.control_speed_theta = 0
        self.current_speed_x = 0
        self.current_speed_theta = 0
        self.visual_status_time = int(time.time() * 1000)
        self.gc_status_time = int(time.time() * 1000)
        self.gba_status_time = int(time.time() * 1000)
        self.charge_status_time = int(time.time() * 1000)
        self.power_time = int(time.time() * 1000)
        self.control_speed_time = int(time.time() * 1000)
        self.current_speed_time = int(time.time() * 1000)
        self.galileo_status = galileo_status
        self.galileo_status_lock = galileo_status_lock
        self.listener = tf.TransformListener(True, rospy.Duration(10.0))
        self.nav_action_cache = None
        self.nav_action_time = 0
        self.loop_task_cache = None
        self.loop_task_time = 0

        # stop flag
        self._stop = threading.Event()
        self._stop.set()

        def update_visual_status(status):
            self.visual_status_time = int(time.time() * 1000)
            if status.data <= 1:
                self.visual_status = 0
            if status.data == 2:
                self.visual_status = 1
            if status.data == 3:
                self.visual_status = 2

        def update_gc_status(status):
            self.gc_status_time = int(time.time() * 1000)
            if status.data:
                self.gc_status = 1
            else:
                self.gc_status = 0

        def update_gba_status(status):
            self.gba_status_time = int(time.time() * 1000)
            if status.data:
                self.gba_status = 1
            else:
                self.gba_status = 0

        def update_charge_status(status):
            self.charge_status_time = int(time.time() * 1000)
            self.charge_status = status.data

        def update_power(power):
            if power.data < POWER_LOW / 2:
                return
            self.power_time = int(time.time() * 1000)
            self.power = power.data

        def update_current_speed(odom):
            self.current_speed_time = int(time.time() * 1000)
            self.current_speed_x = odom.twist.twist.linear.x
            self.current_speed_theta = odom.twist.twist.angular.z

        def update_control_speed(twist):
            self.control_speed_time = int(time.time() * 1000)
            self.control_speed_x = twist.linear.x
            self.control_speed_theta = twist.angular.z

        self.visual_sub = rospy.Subscriber(
            "/ORB_SLAM/TrackingStatus", Int32, update_visual_status)
        self.gc_sub = rospy.Subscriber("/ORB_SLAM/GC", Bool, update_gc_status)
        self.gba_sub = rospy.Subscriber(
            "/ORB_SLAM/GBA", Bool, update_gba_status)
        self.charge_sub = rospy.Subscriber(
            "/bw_auto_dock/Chargestatus", Int32, update_charge_status)

        # self.power_sub = rospy.Subscriber(
        #     "/bw_auto_dock/Batterypower", Float32, update_power)
        self.power_sub2 = rospy.Subscriber(
            "/xqserial_server/Power", Float64, update_power)
        self.current_speed_sub = rospy.Subscriber(
            "/cmd_vel", Twist, update_control_speed)
        self.control_speed_sub = rospy.Subscriber(
            "/xqserial_server/Odom", Odometry, update_current_speed)

    def stop(self):
        self.visual_sub.unregister()
        self.gc_sub.unregister()
        self.gba_sub.unregister()
        self.charge_sub.unregister()
        # self.power_sub.unregister()
        self.power_sub2.unregister()
        self.current_speed_sub.unregister()
        self.control_speed_sub.unregister()
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()

    def run(self):
        rate = rospy.Rate(30)
        self._stop.clear()
        while not self.stopped() and not rospy.is_shutdown():
            with self.galileo_status_lock:
                nav_task = self.monitor_server.nav_task
                if nav_task is not None:
                    self.galileo_status.navStatus = 1
                else:
                    self.galileo_status.navStatus = 0
                self.galileo_status.visualStatus = self.visual_status
                if self.monitor_server.map_thread.stopped():
                    self.galileo_status.mapStatus = 0
                else:
                    self.galileo_status.mapStatus = 1
                self.galileo_status.gcStatus = self.gc_status
                self.galileo_status.gbaStatus = self.gba_status
                self.galileo_status.chargeStatus = self.charge_status
                self.galileo_status.loopStatus = 0
                if nav_task is not None:
                    self.galileo_status.loopStatus = nav_task.loop_running_flag
                self.galileo_status.power = self.power
                self.galileo_status.targetNumID = -1
                if nav_task is not None:
                    self.galileo_status.targetNumID = nav_task.current_goal_id

                self.galileo_status.targetStatus = 0
                self.galileo_status.angleGoalStatus = 1
                if nav_task is not None:
                    if nav_task.current_goal_status() == "FREE":
                        self.galileo_status.targetStatus = 0
                        self.galileo_status.angleGoalStatus = 1
                    if nav_task.current_goal_status() == "WORKING":
                        self.galileo_status.targetStatus = 1
                        self.galileo_status.angleGoalStatus = 0
                    if nav_task.current_goal_status() == "PAUSED":
                        self.galileo_status.targetStatus = 2
                    if nav_task.current_goal_status() == "ERROR":
                        self.galileo_status.targetStatus = -1

                self.galileo_status.targetDistance = -1
                if nav_task is not None and \
                        nav_task.current_goal_status() != "ERROR":
                    self.galileo_status.targetDistance = \
                        nav_task.current_goal_distance()
                    if nav_task.update_pose() != -1:
                        self.galileo_status.currentPosX = nav_task.current_pose_stamped_map.pose.position.x
                        self.galileo_status.currentPosY = nav_task.current_pose_stamped_map.pose.position.y
                        current_oritation = nav_task.current_pose_stamped_map.pose.orientation
                        current_pose_q = [current_oritation.x, current_oritation.y,
                                        current_oritation.z, current_oritation.w]
                        self.galileo_status.currentAngle = euler_from_quaternion(current_pose_q)[
                            2]
                    else:
                        self.galileo_status.currentPosX = -1
                        self.galileo_status.currentPosY = -1
                        self.galileo_status.currentAngle = -1
                elif not self.monitor_server.map_thread.stopped() and self.visual_status >= 1:
                    # 处于建图状态
                    # 获取map到baselink坐标变换
                    latest = rospy.Time(0)
                    current_pose_stamped = PoseStamped()
                    current_pose_stamped.header.stamp = latest
                    current_pose_stamped_map = None
                    try:
                        current_pose_stamped.header.frame_id = "base_link"
                        current_pose_stamped_map = self.listener.transformPose(
                            "map", current_pose_stamped)
                    except (tf.LookupException, tf.ConnectivityException,
                            tf.ExtrapolationException, tf.Exception) as e:
                        rospy.logwarn(e)
                    if current_pose_stamped_map == None:
                        self.galileo_status.currentPosX = -1
                        self.galileo_status.currentPosY = -1
                        self.galileo_status.currentAngle = -1
                    else:
                        self.galileo_status.currentPosX = current_pose_stamped_map.pose.position.x
                        self.galileo_status.currentPosY = current_pose_stamped_map.pose.position.y
                        current_oritation = current_pose_stamped_map.pose.orientation
                        current_pose_q = [current_oritation.x, current_oritation.y,
                                        current_oritation.z, current_oritation.w]
                        self.galileo_status.currentAngle = euler_from_quaternion(current_pose_q)[
                            2]
                else:
                    self.galileo_status.currentPosX = -1
                    self.galileo_status.currentPosY = -1
                    self.galileo_status.currentAngle = -1

                self.galileo_status.controlSpeedX = self.control_speed_x
                self.galileo_status.controlSpeedTheta = self.control_speed_theta
                self.galileo_status.currentSpeedX = self.current_speed_x
                self.galileo_status.currentSpeedTheta = self.current_speed_theta
                self.galileo_status.header.frame_id = "map"
                self.galileo_status.header.stamp = rospy.Time.now()

                self.galileo_status.busyStatus = 0
                if self.monitor_server.busy_flag:
                    self.galileo_status.busyStatus = 1

                # 兼容http api状态，根据galieo api任务状态更新galileo status
                if self.galileo_status.navStatus == 1:
                    nav_task_info = self.get_current_nav_action()
                    if nav_task_info is not None:
                        self.galileo_status.targetNumID = -2
                        if nav_task_info["state"] == "WORKING":
                            self.galileo_status.targetStatus = 1
                            self.galileo_status.angleGoalStatus = 0
                        if nav_task_info["state"] == "PAUSED":
                            self.galileo_status.targetStatus = 2
                            self.galileo_status.angleGoalStatus = 2
                        if "index" in nav_task_info and nav_task_info["index"] != -1:
                            self.galileo_status.targetNumID = nav_task_info["index"]
                        self.galileo_status.targetDistance = nav_task_info["current_distance"]
                    task_info = self.get_last_loop_task()
                    if task_info is not None:
                        if task_info["state"] == "CANCELLED" or task_info["state"] == "ERROR" or task_info["state"] == "COMPLETE":
                            self.galileo_status.loopStatus = 0    
                        else:
                            self.galileo_status.loopStatus = 1
                    else:
                        self.galileo_status.loopStatus = 0

                # reset old status
                now = int(time.time() * 1000)
                if now - self.visual_status_time > 1000:
                    self.visual_status = -1 # 视觉程序未开始运行
                    self.galileo_status.visualStatus = -1
                if now - self.gc_status_time > 1000:
                    self.gc_status = 0
                    self.galileo_status.gcStatus = 0
                if now - self.gba_status_time > 1000:
                    self.gba_status = 0
                    self.galileo_status.gbaStatus = 0
                if now - self.charge_status_time > 1000:
                    self.charge_status = 0
                    self.galileo_status.chargeStatus = 0
                if now - self.power_time > 1000:
                    self.power = 0
                    self.galileo_status.power = 0
                if now - self.control_speed_time > 1000:
                    self.control_speed_theta = 0
                    self.control_speed_x = 0
                    self.galileo_status.controlSpeedX = 0
                    self.galileo_status.controlSpeedTheta = 0
                if now - self.current_speed_time > 1000:
                    self.current_speed_theta = 0
                    self.current_speed_x = 0
                    self.galileo_status.currentSpeedX = 0
                    self.galileo_status.currentSpeedTheta = 0

            self.galileo_pub.publish(self.galileo_status)
            rate.sleep()

    def get_current_nav_action(self):
        if time.time() - self.nav_action_time > 1:
            try:
                self.nav_action_time = time.time()
                res = requests.get("http://127.0.0.1:3546/api/v1/task?id=current_nav_action")
                if res.status_code == 200:
                    # 正在执行 nav action
                    self.nav_action_cache = json.loads(res.content.decode("utf-8"))
                else:
                    self.nav_action_cache = None
            except Exception:
                self.nav_action_cache = None
        return self.nav_action_cache
            

    def get_last_loop_task(self):
        if time.time() - self.loop_task_time > 1:
            try:
                self.loop_task_time = time.time()
                res = requests.get("http://127.0.0.1:3546/api/v1/navigation/loop_task")
                if res.status_code == 200:
                    # 正在执行 nav action
                    self.loop_task_cache = json.loads(res.content.decode("utf-8"))
                else:
                    self.loop_task_cache = None
            except Exception:
                self.loop_task_cache = None
        return self.loop_task_cache

        