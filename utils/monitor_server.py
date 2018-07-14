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
# Author: Randoms, Xie fusheng
#

import commands
import os
import struct
import threading
import time
from socket import AF_INET, SOCK_DGRAM, socket, timeout

import numpy as np
import psutil
import rospy
import tf
from galileo_serial_server.msg import GalileoNativeCmds, GalileoStatus
from geometry_msgs.msg import Pose, Pose2D, PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64, Int16, String, UInt32
from system_monitor.msg import Status

from config import MAX_THETA, MAX_VEL, ROS_PACKAGE_PATH
from map_service import MapService
from nav_task import NavigationTask
from navigation_service import NavigationService
from req_parser import ReqParser
from utils import stop_process


class MonitorServer(threading.Thread):
    # 接收udp命令的socket
    def __init__(self, pubs, robot_status_lock, robot_status,
                 host='', user_socket_port=20001, buf_size=1024):
        super(MonitorServer, self).__init__()
        self.host = host
        self.usersocket_port = user_socket_port
        self.buf_size = buf_size
        self._stop = threading.Event()
        self.user_server_socket = socket(AF_INET, SOCK_DGRAM)
        self.user_server_socket.bind((self.host, self.usersocket_port))
        self.parser = ReqParser()
        self.speed_cmd = Twist()

        self.global_move_pub = pubs["GLOBAL_MOVE_PUB"]
        self.cmd_vel_pub = pubs["CMD_VEL_PUB"]
        self.map_save_pub = pubs["MAPSAVE_PUB"]
        self.elevator_pub = pubs["ELEVATOR_PUB"]
        self.tilt_pub = pubs["TILT_PUB"]
        self.charge_pub = pubs["CHARGE_PUB"]
        self.charge_pose_pub = pubs["CHARGE_POSE_PUB"]

        self.map_thread = MapService(robot_status_lock, robot_status)
        self.nav_thread = NavigationService(robot_status_lock, robot_status)
        self.last_nav_time = rospy.Time(0)
        self.nav_flag = False
        self.user_socket_remote = None

        self.nav_task = None
        rospy.loginfo("service started")

        def get_galileo_cmds(cmds):
            self.parse_data([map(lambda x: ord(x), list(cmds.data))])

        rospy.Subscriber('/galileo/cmds', GalileoNativeCmds, get_galileo_cmds)

    def stop(self):
        if self.user_server_socket != None:
            self.user_server_socket.close()
        if not self.map_thread.stopped():
            self.map_thread.stop()
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()

    def run(self):
        self.user_server_socket.settimeout(2)  # 设置udp 2秒超时 等待
        while not self.stopped() and not rospy.is_shutdown():
            try:
                data, self.user_socket_remote = self.user_server_socket.recvfrom(
                    self.buf_size)
            except timeout:
                continue
            if not data:
                break
            dataList = []
            for c in data:
                dataList.append(ord(c))
            self.parse_data(self.parser.unpack_req(dataList))  # 处理命令数据
        self.stop()

    def parse_data(self, cmds):
        res = None
        time_now = rospy.Time.now()
        for count in range(0, len(cmds)):
            if len(cmds[count]) > 0:
                self.CONTROL_FLAG = True
                global_move_flag = Bool()
                global_move_flag.data = True
                self.global_move_pub.publish(global_move_flag)
            if len(cmds[count]) == 2:
                global_move_flag = Bool()
                global_move_flag.data = True

                temp_scale = 1.0
                if abs(self.speed_cmd.linear.x) < 1.0:
                    temp_scale = 1.0
                else:
                    temp_scale = abs(self.speed_cmd.linear.x)

                # 判断是否为关机命令
                if cmds[count][0] == 0xaa and cmds[count][1] == 0x44:
                    rospy.loginfo("system poweroff")
                    commands.getstatusoutput(
                        'sudo shutdown -h now')

                if cmds[count][0] == ord('f'):
                    rospy.loginfo("forward")
                    self.global_move_pub.publish(global_move_flag)
                    self.speed_cmd.linear.x = MAX_VEL * cmds[count][1] / 100.0
                    self.cmd_vel_pub.publish(self.speed_cmd)
                elif cmds[count][0] == ord('b'):
                    rospy.loginfo("back")
                    self.global_move_pub.publish(global_move_flag)
                    self.speed_cmd.linear.x = -MAX_VEL * cmds[count][1] / 100.0
                    self.cmd_vel_pub.publish(self.speed_cmd)
                elif cmds[count][0] == ord('c'):
                    rospy.loginfo("circleleft")
                    self.global_move_pub.publish(global_move_flag)
                    if cmds[count][1] > 1:
                        self.speed_cmd.angular.z = max(
                            0.4, MAX_THETA * cmds[count][1] / 100.0 / temp_scale)
                    else:
                        self.speed_cmd.angular.z = MAX_THETA * \
                            cmds[count][1] / 100.0 / temp_scale
                    #self.SPEED_CMD.angular.z = MAX_THETA * cmds[count][1]/100.0/2.8
                    self.cmd_vel_pub.publish(self.speed_cmd)
                elif cmds[count][0] == ord('d'):
                    rospy.loginfo("circleright")
                    self.global_move_pub.publish(global_move_flag)
                    if cmds[count][1] > 1:
                        self.speed_cmd.angular.z = min(
                            -0.4, -MAX_THETA * cmds[count][1] / 100.0 / temp_scale)
                    else:
                        self.speed_cmd.angular.z = -MAX_THETA * \
                            cmds[count][1] / 100.0 / temp_scale
                    #self.SPEED_CMD.angular.z = -MAX_THETA * cmds[count][1]/100.0/2.8
                    self.cmd_vel_pub.publish(self.speed_cmd)
                elif cmds[count][0] == ord('s'):
                    rospy.loginfo("stop")
                    self.speed_cmd.linear.x = 0
                    self.speed_cmd.angular.z = 0
                    self.cmd_vel_pub.publish(self.speed_cmd)
                elif cmds[count][0] == ord('V'):
                    if cmds[count][1] == 0:
                        rospy.loginfo("开启视觉")
                        if self.map_thread.stopped():
                            rospy.loginfo("开启视觉2")
                            self.map_thread.update = False
                            self.map_thread.start()
                    elif cmds[count][1] == 1:
                        rospy.loginfo("关闭视觉")
                        if not self.map_thread.stopped():
                            rospy.loginfo("关闭视觉2")
                            self.map_thread.stop()
                        os.system("pkill -f getORBtrack.py")
                        os.system("pkill -f navGuide.py")
                        os.system("pkill -f ORB_SLAM")
                        os.system("pkill -f map_server")
                        os.system("pkill -f move_base")
                        os.system("pkill -f odom_map_broadcaster")
                    elif cmds[count][1] == 2:
                        rospy.loginfo("保存地图")
                        mapSaveFlag = Bool()
                        mapSaveFlag.data = True
                        self.map_save_pub.publish(mapSaveFlag)
                        if self.map_thread.scale_orb_thread != None:
                            self.map_thread.scale_orb_thread.save_scale()
                    elif cmds[count][1] == 3:
                        rospy.loginfo("更新地图")
                        if self.map_thread.stopped():
                            rospy.loginfo("开启视觉2")
                            self.map_thread.update = True
                            self.map_thread.start()
                elif cmds[count][0] == ord('h'):
                    elePose = UInt32()
                    elePose.data = cmds[count][1]
                    self.elevator_pub.publish(elePose)
                elif cmds[count][0] == ord('m'):
                    time1_diff = time_now - self.last_nav_time
                    if cmds[count][1] == 1:
                        if time1_diff.to_sec() < 30:
                            continue
                        rospy.loginfo("开始低速巡检")
                        self.last_nav_time = time1_diff
                        tilt_degree = Int16()
                        tilt_degree.data = -19
                        self.tilt_pub.publish(tilt_degree)
                        if self.nav_thread.stopped():
                            self.nav_thread.setspeed(1)
                            self.nav_thread.start()
                            self.nav_task = NavigationTask()

                    if cmds[count][1] == 2:
                        if time1_diff.to_sec() < 30:
                            continue
                        rospy.loginfo("开始中速巡检")
                        self.last_nav_time = time1_diff
                        tilt_degree = Int16()
                        tilt_degree.data = -19
                        self.tilt_pub.publish(tilt_degree)
                        if self.nav_thread.stopped():
                            self.nav_thread.setspeed(2)
                            self.nav_thread.start()
                            self.nav_task = NavigationTask()
                    if cmds[count][1] == 3:
                        if time1_diff.to_sec() < 30:
                            continue
                        rospy.loginfo("开始高速巡检")
                        self.last_nav_time = time1_diff
                        tilt_degree = Int16()
                        tilt_degree.data = -19
                        self.tilt_pub.publish(tilt_degree)
                        if self.nav_thread.stopped():
                            self.nav_thread.setspeed(3)
                            self.nav_thread.start()
                            self.nav_task = NavigationTask()
                    if cmds[count][1] == 0:
                        if time1_diff.to_sec() < 30:
                            continue
                        rospy.loginfo("开启视觉，不巡检")
                        self.last_nav_time = time1_diff
                        tilt_degree = Int16()
                        tilt_degree.data = -19
                        self.tilt_pub.publish(tilt_degree)
                        if self.nav_thread.stopped():
                            self.nav_thread.setspeed(0)
                            self.nav_thread.start()
                            self.nav_task = NavigationTask()
                    if cmds[count][1] == 4:
                        rospy.loginfo("关闭自主巡检")
                        if self.nav_task is not None:
                            self.nav_task.shutdown()
                            self.nav_task = None
                        tilt_degree = Int16()
                        tilt_degree.data = 0
                        self.tilt_pub.publish(tilt_degree)
                        if not self.nav_thread.stopped():
                            self.nav_thread.stop()
                        self.speed_cmd.linear.x = 0
                        self.speed_cmd.angular.z = 0
                        self.cmd_vel_pub.publish(self.speed_cmd)
                        self.nav_flag = False
                        os.system("pkill -f getORBtrack.py")
                        os.system("pkill -f navGuide.py")
                        os.system("pkill -f ORB_SLAM")
                        os.system("pkill -f map_server")
                        os.system("pkill -f move_base")
                        os.system("pkill -f odom_map_broadcaster")
                elif cmds[count][0] == ord('g'):
                    self.nav_task.set_goal(cmds[count][1])
                elif cmds[count][0] == ord('i'):
                    if cmds[count][1] == 0:
                        self.nav_task.pause()
                    if cmds[count][1] == 1:
                        self.nav_task.resume()
                    if cmds[count][1] == 2:
                        self.nav_task.cancel_goal()
                elif cmds[count][0] == ord('j'):
                    if cmds[count][1] == 0:
                        # start charge
                        if self.nav_task is not None:
                            charge_msg = Bool()
                            charge_msg.data = True
                            self.charge_pub.publish(charge_msg)

                    if cmds[count][1] == 1:
                        # stop charge
                        charge_msg = Bool()
                        charge_msg.data = False
                        self.charge_pub.publish(charge_msg)

                    if cmds[count][1] == 2:
                        # save charge position
                        save_msg = Bool()
                        save_msg.data = True
                        self.charge_pose_pub.publish(save_msg)
            if len(cmds[count]) > 2:
                # insert new goal
                if cmds[count][0] == ord("g") and cmds[count][1] == ord("i"):
                    pos_x = struct.unpack("f", bytearray(cmds[count][2:6]))[0]
                    pos_y = struct.unpack("f", bytearray(cmds[count][6:10]))[0]
                    if self.nav_task is not None:
                        self.nav_task.insert_goal(pos_x, pos_y, 0)
                # reset goals
                if cmds[count][0] == ord("g") and cmds[count][1] == ord("r"):
                    if self.nav_task is not None:
                        self.nav_task.reset_goals()

        return res

    def sendto(self, data):
        try:
            if self.get_connection_status():
                self.user_server_socket.sendto(
                    bytes(data), self.user_socket_remote)
        except:
            rospy.logwarn("remote disconnect !")

    def get_connection_status(self):
        if self.user_server_socket != None and self.user_socket_remote != None:
            return True
        else:
            return False
