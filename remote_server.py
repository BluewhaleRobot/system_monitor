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
import struct
import subprocess
import threading
from socket import AF_INET, SO_BROADCAST, SOCK_DGRAM, SOL_SOCKET, socket

import numpy as np
import psutil
import rospy
import tf
from galileo_serial_server.msg import GalileoNativeCmds, GalileoStatus
from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64, Int16, Int32, UInt32
from system_monitor.msg import Status
from tf.transformations import euler_from_quaternion

from utils.config import BROADCAST_PORT, ROS_PACKAGE_PATH, TF_ROT, TF_TRANS
from utils.galileo_status_service import GalileoStatusService
from utils.monitor_server import MonitorServer
from utils.udp_status_service import UDPStatusService
import rosservice


TILT_PUB = None
rplidar_flag = False

def get_global_move_flag(moveEn):
    if not moveEn.data:
        # 关闭视觉导航
        if not monitor_server.nav_thread.stopped():
            monitor_server.nav_thread.stop()
            if monitor_server.nav_task is not None:
                monitor_server.nav_task.shutdown()
                monitor_server.nav_task = None

def get_scan(scan):
    global rplidar_flag
    rplidar_flag = True

def init_sub_pubs():
    rospy.init_node("remote_server", anonymous=True)
    rospy.Subscriber("/global_move_flag", Bool, get_global_move_flag)
    rospy.Subscriber('/scan', LaserScan, get_scan)
    GLOBAL_MOVE_PUB = rospy.Publisher('/global_move_flag', Bool, queue_size=1)
    ELEVATOR_PUB = rospy.Publisher('/elevatorPose', UInt32, queue_size=1)
    CMD_VEL_PUB = rospy.Publisher('/cmd_vel', Twist, queue_size=0)
    MAPSAVE_PUB = rospy.Publisher('/map_save', Bool, queue_size=0)
    TILT_PUB = rospy.Publisher('/set_tilt_degree', Int16, queue_size=0)
    CHARGE_PUB = rospy.Publisher(
        '/bw_auto_dock/EnableCharge', Bool, queue_size=0)
    CHARGE_POSE_PUB = rospy.Publisher(
        '/bw_auto_dock/dockposition_save', Bool, queue_size=0)
    GALILEO_STATUS_PUB = rospy.Publisher(
        '/galileo/status', GalileoStatus, queue_size=0)
    return {
        "GLOBAL_MOVE_PUB": GLOBAL_MOVE_PUB,
        "ELEVATOR_PUB": ELEVATOR_PUB,
        "CMD_VEL_PUB": CMD_VEL_PUB,
        "MAPSAVE_PUB": MAPSAVE_PUB,
        "TILT_PUB": TILT_PUB,
        "GALILEO_STATUS_PUB": GALILEO_STATUS_PUB,
        "CHARGE_PUB": CHARGE_PUB,
        "CHARGE_POSE_PUB": CHARGE_POSE_PUB,
    }


if __name__ == "__main__":
    pubs = init_sub_pubs()
    rate = rospy.Rate(10)
    # 配置udp广播
    s = socket(AF_INET, SOCK_DGRAM)
    s.bind(('', 0))
    s.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)
    
    # 配置全局系统状态
    galileo_status = GalileoStatus()
    galileo_status_lock = threading.Lock()

    # 伽利略指令处理线程
    monitor_server = MonitorServer(pubs, galileo_status, galileo_status_lock)
    monitor_server.start()

    # 伽利略系统状态发布线程
    galileo_status_service = GalileoStatusService(
        pubs["GALILEO_STATUS_PUB"], monitor_server, galileo_status, galileo_status_lock)
    galileo_status_service.start()

    # UDP系统状态发布线程
    udp_status_service = UDPStatusService(monitor_server, galileo_status_service.galileo_status,
                                          galileo_status_service.galileo_status_lock)
    udp_status_service.start()

    broadcast_count = 10  # 每1秒播放一次声音
    heart_beat_count = 40  # 每4秒心跳维护一次

    sub_process_thread = None
    sub_process_thread_ps_process = None
    while not rospy.is_shutdown():
        if heart_beat_count == 40:
            heart_beat_count = 0
        heart_beat_count += 1

        if heart_beat_count == 39:
            new_env = os.environ.copy()
            new_env['ROS_PACKAGE_PATH'] = ROS_PACKAGE_PATH
            if sub_process_thread != None:
                sub_process_thread_ps_process = psutil.Process(
                    pid=sub_process_thread.pid)
                for child in sub_process_thread_ps_process.children(recursive=True):
                    child.kill()
                sub_process_thread_ps_process.kill()
                sub_process_thread = None
            else:
                cmd = None
                if galileo_status.navStatus == 1 and not rplidar_flag:
                    # 打开雷达电机
                    if rosservice.get_service_node("/start_motor") is not None:
                        cmd = "rosservice call /start_motor"
                        sub_process_thread = subprocess.Popen(
                            cmd, shell=True, env=new_env)
                elif rplidar_flag and galileo_status.navStatus == 0:
                    # 关闭雷达电机
                    if rosservice.get_service_node("/stop_motor") is not None:
                        cmd = "rosservice call /stop_motor"
                        sub_process_thread = subprocess.Popen(
                            cmd, shell=True, env=new_env)
        if heart_beat_count == 1:
            rplidar_flag = False

        # 每秒广播一次
        if broadcast_count == 10:
            broadcast_count = 0
            data = "xq"
            # 发送广播包
            try:
                s.sendto(data, ('<broadcast>', BROADCAST_PORT))
            except:
                continue
        broadcast_count += 1
        rate.sleep()
    monitor_server.stop()
    galileo_status_service.stop()
    udp_status_service.stop()
