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

from subprocess import Popen
import os
import struct
import threading
import time
from socket import AF_INET, SOCK_DGRAM, socket, timeout

import numpy as np
import psutil
import rospy
import tf
import rosservice
from galileo_serial_server.msg import GalileoNativeCmds, GalileoStatus
from geometry_msgs.msg import Pose, Pose2D, PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64, Int16, String, UInt32, String
from system_monitor.msg import Status
from dynamic_reconfigure.server import Server

from .config import MAX_THETA, MAX_VEL, ROS_PACKAGE_PATH
from .map_service import MapService
from .nav_task import NavigationTask
from .schedule_nav_task import ScheduleNavTask
from .navigation_service import NavigationService
from .schedule_service import ScheduleService
from .req_parser import ReqParser
from .utils import stop_process
from system_monitor.cfg import system_monitorConfigfrom system_monitor.cfg import system_monitorConfig


class MonitorServer(threading.Thread):
    # 接收udp命令的socket
    def __init__(self, pubs, galileo_status, galileo_status_lock,
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
        self.audio_pub = pubs["AUDIO_PUB"]
        self.poweroff_pub = pubs["POWEROFF_PUB"]

        self.galileo_status = galileo_status
        self.galileo_status_lock = galileo_status_lock

        self.map_thread = MapService(self.galileo_status, self.galileo_status_lock)
        self.nav_thread = NavigationService(self.galileo_status, self.galileo_status_lock)
        self.last_nav_time = rospy.Time(0)
        self.user_socket_remote = None

        self.nav_task = None
        self.nav_guide_task = None
        self.busy_flag = False

        # 控制最大遥控直线速度和转动速度
        self.max_vel = MAX_VEL
        self.max_theta = MAX_THETA

        def update_params(config, level):
            self.max_theta = config.max_theta
            self.max_vel = config.max_vel
            return config

        self.param_server = Server(system_monitorConfig, update_params)

        rospy.loginfo("service started")

        def get_galileo_cmds(cmds):
            self.parse_data([map(lambda x: ord(x), list(cmds.data))])

        self.cmd_sub = rospy.Subscriber('/galileo/cmds', GalileoNativeCmds, get_galileo_cmds)

    def stop(self):
        self.cmd_sub.unregister()
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
                dataList.append(c)
            self.parse_data(self.parser.unpack_req(dataList))  # 处理命令数据
        self.stop()

    def parse_data(self, cmds):
        # 等待退出busy状态，最长等待3分钟
        wait_busy_count = 3 * 60 * 100
        while self.busy_flag and wait_busy_count > 0:
            time.sleep(0.01)
            wait_busy_count -= 1
        self.busy_flag = True
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
                    self.audio_pub.publish("请等待一分钟后，再切断总电源，谢谢！")
                    # 等待语音播放完毕
                    time.sleep(5)
                    shutdown_cmd = Popen('sudo shutdown -h now', shell=True)
                    shutdown_cmd.wait()

                if cmds[count][0] == ord('f'):
                    rospy.loginfo("forward")
                    self.global_move_pub.publish(global_move_flag)
                    self.speed_cmd.linear.x = self.max_vel * cmds[count][1] / 100.0
                    self.cmd_vel_pub.publish(self.speed_cmd)
                elif cmds[count][0] == ord('b'):
                    rospy.loginfo("back")
                    self.global_move_pub.publish(global_move_flag)
                    self.speed_cmd.linear.x = -self.max_vel * cmds[count][1] / 100.0
                    self.cmd_vel_pub.publish(self.speed_cmd)
                elif cmds[count][0] == ord('c'):
                    rospy.loginfo("circleleft")
                    self.global_move_pub.publish(global_move_flag)
                    if cmds[count][1] > 1:
                        self.speed_cmd.angular.z = max(
                            0.4, self.max_theta * cmds[count][1] / 100.0 / temp_scale)
                    else:
                        self.speed_cmd.angular.z = self.max_theta * \
                            cmds[count][1] / 100.0 / temp_scale
                    self.cmd_vel_pub.publish(self.speed_cmd)
                elif cmds[count][0] == ord('d'):
                    rospy.loginfo("circleright")
                    self.global_move_pub.publish(global_move_flag)
                    if cmds[count][1] > 1:
                        self.speed_cmd.angular.z = min(
                            -0.4, -self.max_theta * cmds[count][1] / 100.0 / temp_scale)
                    else:
                        self.speed_cmd.angular.z = -self.max_theta * \
                            cmds[count][1] / 100.0 / temp_scale
                    self.cmd_vel_pub.publish(self.speed_cmd)
                elif cmds[count][0] == ord('s'):
                    rospy.loginfo("stop")
                    self.speed_cmd.linear.x = 0
                    self.speed_cmd.angular.z = 0
                    self.cmd_vel_pub.publish(self.speed_cmd)
                elif cmds[count][0] == ord('V'):
                    if cmds[count][1] == 0:
                        rospy.loginfo("开始建图")
                        if self.nav_task is not None:
                            rospy.loginfo("停止导航任务")
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
                        os.system("pkill -f 'roslaunch nav_test tank_blank_map0.launch'")
                        os.system("pkill -f 'roslaunch nav_test tank_blank_map1.launch'")
                        os.system("pkill -f 'roslaunch nav_test tank_blank_map2.launch'")
                        os.system("pkill -f 'roslaunch nav_test tank_blank_map3.launch'")

                        if self.map_thread.stopped():
                            rospy.loginfo("启动建图程序")
                            self.map_thread.update = False
                            self.map_thread.start()
                    elif cmds[count][1] == 1:
                        rospy.loginfo("停止建图")
                        if not self.map_thread.stopped():
                            rospy.loginfo("停止建图程序")
                            self.map_thread.stop()
                    elif cmds[count][1] == 2:
                        rospy.logwarn("不再支持串口保存地图协议")
                        continue
                        # rospy.loginfo("保存地图")
                        # mapSaveFlag = Bool()
                        # mapSaveFlag.data = True
                        # self.map_save_pub.publish(mapSaveFlag)
                        # if self.map_thread.scale_orb_thread != None:
                        #     self.map_thread.scale_orb_thread.save_scale()
                    elif cmds[count][1] == 3:
                        rospy.loginfo("更新地图")

                        if self.nav_task is not None:
                            rospy.loginfo("停止导航任务")
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
                        
                        if self.map_thread.stopped():
                            rospy.loginfo("启动更新程序")
                            self.map_thread.update = True
                            self.map_thread.start()
                elif cmds[count][0] == ord('h'):
                    elePose = UInt32()
                    elePose.data = cmds[count][1]
                    self.elevator_pub.publish(elePose)
                elif cmds[count][0] == ord('m'):
                    time1_diff = time_now - self.last_nav_time
                    if cmds[count][1] == 0:
                        if not rospy.get_param("/system_monitor/nav_is_enabled", True):
                            continue
                        if time1_diff.to_sec() < 5:
                            continue
                        rospy.loginfo("开启导航")
                        if not self.map_thread.stopped():
                            rospy.loginfo("关闭建图程序")
                            self.map_thread.stop()

                        self.last_nav_time = time_now
                        tilt_degree = Int16()
                        tilt_degree.data = -19
                        self.tilt_pub.publish(tilt_degree)
                        if self.nav_thread.stopped():
                            rospy.loginfo("开启导航程序")
                            self.nav_thread = NavigationService(self.galileo_status, self.galileo_status_lock)
                            self.nav_thread.start()
                            if self.nav_task is not None:
                                self.nav_task.shutdown()
                            self.nav_task = NavigationTask()
                    # 开启调度导航
                    if cmds[count][1] == 7:
                        if not rospy.get_param("/system_monitor/nav_is_enabled", True):
                            continue
                        if time1_diff.to_sec() < 5:
                            continue
                        rospy.loginfo("开启调度系统")
                        if not self.map_thread.stopped():
                            rospy.loginfo("关闭建图程序")
                            self.map_thread.stop()

                        self.last_nav_time = time_now
                        tilt_degree = Int16()
                        tilt_degree.data = -19
                        self.tilt_pub.publish(tilt_degree)
                        if self.nav_thread.stopped():
                            rospy.loginfo("开启调度导航程序")
                            self.nav_thread = ScheduleService(self.galileo_status, self.galileo_status_lock)
                            self.nav_thread.start()
                            if self.nav_task is not None:
                                self.nav_task.shutdown()
                            self.nav_task = ScheduleNavTask()
                    if cmds[count][1] == 4 or cmds[count][1] == 8:
                        rospy.loginfo("关闭导航系统")
                        if not self.map_thread.stopped():
                            rospy.loginfo("关闭建图程序")
                            self.map_thread.stop()

                        if self.nav_task is not None:
                            rospy.loginfo("关闭导航任务程序")
                            self.nav_task.shutdown()
                            self.nav_task = None
                        tilt_degree = Int16()
                        tilt_degree.data = 0
                        self.tilt_pub.publish(tilt_degree)
                        if not self.nav_thread.stopped():
                            rospy.loginfo("关闭导航程序")
                            self.nav_thread.stop()
                            if self.nav_task is not None:
                                self.nav_task.shutdown()
                        self.speed_cmd.linear.x = 0
                        self.speed_cmd.angular.z = 0
                        self.cmd_vel_pub.publish(self.speed_cmd)
                    if cmds[count][1] == 5:
                        if not rospy.get_param("/system_monitor/nav_is_enabled", True):
                            continue
                        rospy.loginfo("开启自动巡检")
                        if not self.map_thread.stopped():
                            rospy.loginfo("关闭建图程序")
                            self.map_thread.stop()

                        tilt_degree = Int16()
                        tilt_degree.data = -19
                        self.tilt_pub.publish(tilt_degree)
                        if self.nav_thread.stopped():
                            rospy.logwarn("导航程序未启动")
                            self.busy_flag = False
                            return
                        if self.nav_task.loop_running_flag:
                            rospy.logwarn("已经开始巡检")
                            self.busy_flag = False
                            return
                        rospy.loginfo("启动巡检程序")
                        self.nav_task.start_loop()
                    if cmds[count][1] == 6:
                        rospy.loginfo("停止自动巡检")
                        if self.nav_task is not None:
                            self.nav_task.stop_loop()
                    
                    if cmds[count][1] == 9:
                        # 重载导航路径
                        # 关闭导航任务
                        if self.nav_thread is None:
                            continue
                        self.nav_thread.reload()
                        if self.nav_task is not None:
                            self.nav_task.shutdown()
                        # 重新载入导航任务
                        self.nav_task = NavigationTask()
                        tilt_degree = Int16()
                        tilt_degree.data = 0
                        self.tilt_pub.publish(tilt_degree)
                        self.speed_cmd.linear.x = 0
                        self.speed_cmd.angular.z = 0
                        self.cmd_vel_pub.publish(self.speed_cmd)
                    
                    if cmds[count][1] == 10:
                        # 重载调度导航路径
                        # 关闭导航任务
                        if self.nav_thread is None:
                            continue
                        self.nav_thread.reload()
                        if self.nav_task is not None:
                            self.nav_task.shutdown()
                        # 重新载入导航任务
                        self.nav_task = ScheduleNavTask()
                        tilt_degree = Int16()
                        tilt_degree.data = 0
                        self.tilt_pub.publish(tilt_degree)
                        self.speed_cmd.linear.x = 0
                        self.speed_cmd.angular.z = 0
                        self.cmd_vel_pub.publish(self.speed_cmd)


                elif cmds[count][0] == ord('g'):
                    if self.nav_task is None:
                        self.busy_flag = False
                        return
                    self.nav_task.set_goal(cmds[count][1])
                elif cmds[count][0] == ord('i'):
                    if self.nav_task is None:
                        self.busy_flag = False
                        return
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
                elif cmds[count][0] == ord('H'):
                    if cmds[count][1] == 0:
                        # start greetings
                        rospy.set_param("/xiaoqiang_greeting_node/is_enabled", True)
                    if cmds[count][1] == 1:
                        # stop greetings
                        rospy.set_param("/xiaoqiang_greeting_node/is_enabled", False)

            if len(cmds[count]) > 2:
                # insert new goal
                if cmds[count][0] == ord("g") and cmds[count][1] == ord("i"):
                    pos_x = struct.unpack("f", bytearray(cmds[count][2:6]))[0]
                    pos_y = struct.unpack("f", bytearray(cmds[count][6:10]))[0]
                    print("######################")
                    print("inser goal")
                    if self.nav_task is not None:
                        rospy.logwarn("插入目标点: " +  str(pos_x) + " " + str(pos_y))
                        self.nav_task.insert_goal(pos_x, pos_y, 0)
                # reset goals
                if cmds[count][0] == ord("g") and cmds[count][1] == ord("r"):
                    if self.nav_task is not None:
                        self.nav_task.reset_goals()

                # set loop and sleep time
                if cmds[count][0] == ord("m") and cmds[count][1] == 5:
                    if self.nav_thread.stopped():
                        rospy.logwarn("巡检状态未启动")
                        self.busy_flag = False
                        return
                    if self.nav_task is not None:
                        if not self.nav_task.loop_running_flag:
                            tilt_degree = Int16()
                            tilt_degree.data = -19
                            self.tilt_pub.publish(tilt_degree)
                            self.nav_task.sleep_time = cmds[count][2]
                            self.nav_task.start_loop()
                        else:
                            self.nav_task.sleep_time = cmds[count][2]
        self.busy_flag = False

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
