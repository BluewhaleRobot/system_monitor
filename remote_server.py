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

import struct
import threading
from socket import AF_INET, SO_BROADCAST, SOCK_DGRAM, SOL_SOCKET, socket

import numpy as np
import rospy
import tf
from galileo_serial_server.msg import GalileoNativeCmds, GalileoStatus
from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64, Int16, Int32, UInt32
from system_monitor.msg import Status

from utils.config import BROADCAST_PORT, TF_ROT, TF_TRANS
from utils.monitor_server import MonitorServer

ROBOT_STATUS = Status()
ROBOT_STATUS.brightness = 0.0
ROBOT_STATUS.imageStatus = False
ROBOT_STATUS.odomStatus = False
ROBOT_STATUS.orbStartStatus = False
ROBOT_STATUS.orbInitStatus = False
ROBOT_STATUS.power = 0.0
ROBOT_STATUS.orbScaleStatus = False

ROBOT_CONTROL_TWIST = None
ROBOT_REAL_TWIST = None

ROBOT_STATUS_LOCK = threading.Lock()

ROBOT_POSESTAMPED = None
CHARGE_STATUS = UInt32()

SEND_DATA = bytearray(
    [205, 235, 215, 36, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00,])

CONTROL_FLAG = False

TILT_PUB = None
NAV_LASTTIME = None

T_HF = [0.0, 0.0, 0.0]
Q_HF = [0.0, 0.0, 0.0, 1.0]


def get_power(power):
    with ROBOT_STATUS_LOCK:
        ROBOT_STATUS.power = power.data


def get_image(image):
    with ROBOT_STATUS_LOCK:
        if image != None:
            ROBOT_STATUS.imageStatus = True
        else:
            ROBOT_STATUS.imageStatus = False


def get_odom(odom):
    global ROBOT_POSESTAMPED, ROBOT_REAL_TWIST, ROBOT_STATUS
    with ROBOT_STATUS_LOCK:
        if odom != None:
            ROBOT_STATUS.odomStatus = True
            ROBOT_POSESTAMPED = PoseStamped()
            ROBOT_POSESTAMPED.pose = odom.pose.pose  # 更新坐标
            ROBOT_POSESTAMPED.header = odom.header  # 更新坐标
            ROBOT_REAL_TWIST = odom.twist
        else:
            ROBOT_STATUS.odomStatus = False


def get_cmd_vel(twist):
    global ROBOT_CONTROL_TWIST
    if twist != None:
        ROBOT_CONTROL_TWIST = twist


def get_orb_start_status(orb_frame):
    with ROBOT_STATUS_LOCK:
        if orb_frame != None:
            ROBOT_STATUS.orbStartStatus = True
        else:
            ROBOT_STATUS.orbStartStatus = False


def get_orb_tracking_flag(cam_pose):
    with ROBOT_STATUS_LOCK:
        if cam_pose != None:
            ROBOT_STATUS.orbInitStatus = True
        else:
            ROBOT_STATUS.orbInitStatus = False


def get_global_move_flag(moveEn):
    if not moveEn.data:
        # 关闭视觉导航
        if not monitor_server.nav_thread.stopped():
            monitor_server.nav_thread.stop()
            monitor_server.nav_flag = False


def get_nav_flag(navRun):
    if navRun.data and not monitor_server.nav_thread.stopped():
        monitor_server.nav_flag = True
    else:
        monitor_server.nav_flag = False


def get_orbgc_status(gc_flag):
    with ROBOT_STATUS_LOCK:
        ROBOT_STATUS.orbGCFlag = gc_flag.data


def get_orbgba_status(gba_flag):
    with ROBOT_STATUS_LOCK:
        ROBOT_STATUS.orbGBAFlag = gba_flag.data


def get_charge_status(charge_status):
    global CHARGE_STATUS
    with ROBOT_STATUS_LOCK:
        CHARGE_STATUS = charge_status


def init_sub_pubs():
    rospy.init_node("remote_server", anonymous=True)
    rospy.Subscriber("/xqserial_server/Power", Float64, get_power)
    rospy.Subscriber("/usb_cam/image_raw", rospy.AnyMsg, get_image)
    rospy.Subscriber("/bWmono/Odom", Odometry, get_odom)
    rospy.Subscriber("/ORB_SLAM/Camera", Pose, get_orb_tracking_flag)
    rospy.Subscriber("/ORB_SLAM/Frame", rospy.AnyMsg, get_orb_start_status)
    rospy.Subscriber("/ORB_SLAM/GC", Bool, get_orbgc_status)
    rospy.Subscriber("/ORB_SLAM/GBA", Bool, get_orbgba_status)
    rospy.Subscriber("/global_move_flag", Bool, get_global_move_flag)
    rospy.Subscriber('/nav_setStop', Bool, get_nav_flag)
    rospy.Subscriber('/cmd_vel', Twist, get_cmd_vel)
    rospy.Subscriber('/bw_auto_dock/Chargestatus', Int32, get_charge_status)
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
    # 开启udp接收监听线程
    monitor_server = MonitorServer(pubs, ROBOT_STATUS_LOCK, ROBOT_STATUS)
    monitor_server.start()
    broadcast_count = 10  # 每1秒播放一次声音
    heart_beat_count = 40  # 每4秒心跳维护一次
    listener = tf.TransformListener(True, rospy.Duration(10.0))
    t_ac = np.array([0., 0., 0.])
    theta_send = 0.
    while not rospy.is_shutdown():

        if heart_beat_count == 40:
            heart_beat_count = 0
            CONTROL_FLAG = False
        heart_beat_count += 1
        # 持续反馈状态
        if monitor_server.get_connection_status():
            if ROBOT_STATUS.odomStatus and ROBOT_POSESTAMPED is not None:
                # 将map坐标系转换成ORB_SLAM/World坐标系
                current_pose = ROBOT_POSESTAMPED.pose
                t_bc = np.array(
                    [current_pose.position.x, current_pose.position.y, current_pose.position.z])
                q = [current_pose.orientation.x, current_pose.orientation.y,
                     current_pose.orientation.z, current_pose.orientation.w]
                m = tf.transformations.quaternion_matrix(q)
                r_bc = m[:3, :3]

                ax, ay, theta_send = tf.transformations.euler_from_matrix(r_bc)
                # 为了简化计算，下文的计算中 base_link 和 base_footprint 被看成是相同的坐标系

                r_ab = TF_ROT.T
                t_ab = -r_ab.dot(TF_TRANS)

                r_ac = r_ab.dot(r_bc)
                t_ac = r_ab.dot(t_bc) + t_ab

            SEND_DATA[4:8] = map(ord, struct.pack('f', t_ac[0]))
            SEND_DATA[8:12] = map(ord, struct.pack('f', t_ac[1]))
            SEND_DATA[12:16] = map(ord, struct.pack('f', t_ac[2]))
            SEND_DATA[16:20] = map(ord, struct.pack('f', ROBOT_STATUS.power))
            SEND_DATA[24:28] = map(ord, struct.pack('f', theta_send))
            if monitor_server.nav_task == None:
                SEND_DATA[28:32] = map(ord, struct.pack('i', 3))
                SEND_DATA[32:36] = map(ord, struct.pack('i', -1))
                SEND_DATA[40:44] = map(ord, struct.pack('i', 0))
            else:
                if monitor_server.nav_task.current_goal_status() == "FREE":
                    SEND_DATA[28:32] = map(ord, struct.pack('i', 0))
                    SEND_DATA[32:36] = map(ord, struct.pack('i', -1))
                if monitor_server.nav_task.current_goal_status() == "WORKING":
                    SEND_DATA[28:32] = map(ord, struct.pack('i', 1))
                    SEND_DATA[32:36] = map(ord, struct.pack('i',
                                                            monitor_server.nav_task.current_goal_id))
                if monitor_server.nav_task.current_goal_status() == "PAUSED":
                    SEND_DATA[28:32] = map(ord, struct.pack('i', 2))
                    SEND_DATA[32:36] = map(ord, struct.pack('i',
                                                            monitor_server.nav_task.current_goal_id))
                SEND_DATA[40:44] = map(ord, struct.pack('i', monitor_server.nav_task.loop_running_flag))
            SEND_DATA[36:40] = map(ord, struct.pack('i', CHARGE_STATUS.data))

            if monitor_server.nav_task != None:
                statu0 = 0x01  # 导航状态
            else:
                statu0 = 0x00
            if ROBOT_STATUS.imageStatus:
                statu1 = 0x02  # 视觉摄像头
            else:
                statu1 = 0x00
            if ROBOT_STATUS.orbStartStatus:
                statu2 = 0x04  # 视觉系统状态
            else:
                statu2 = 0x00
            if ROBOT_STATUS.orbInitStatus:
                statu3 = 0x08  # 视觉系统状态
            else:
                statu3 = 0x00
            if ROBOT_STATUS.orbGCFlag:
                status4 = 0x10  # ORB_SLAM 内存回收状态
            else:
                status4 = 0
            if ROBOT_STATUS.orbGBAFlag:
                status5 = 0x20  # ORB_SLAM2 GBA状态，一般对应LoopClosing
            else:
                status5 = 0x00
            SEND_DATA[20] = statu0 + statu1 + \
                statu2 + statu3 + status4 + status5
        SEND_DATA[3] = len(SEND_DATA) - 4
        monitor_server.sendto(bytes(SEND_DATA))

        # 发布状态topic
        galileo_status = GalileoStatus()
        galileo_status.loopStatus = 0
        if ROBOT_POSESTAMPED is not None:
            galileo_status.header = ROBOT_POSESTAMPED.header
        galileo_status.navStatus = 0
        if not ROBOT_STATUS.orbInitStatus:
            galileo_status.visualStatus = 2
        else:
            galileo_status.visualStatus = 1
        if monitor_server.nav_task != None: # 导航任务正在运行
            galileo_status.navStatus = 1
        else:
            galileo_status.navStatus = 0
        galileo_status.power = ROBOT_STATUS.power
        galileo_status.targetNumID = -1
        if monitor_server.nav_task != None:
            galileo_status.targetNumID = monitor_server.nav_task.current_goal_id
            galileo_status.loopStatus = monitor_server.nav_task.loop_running_flag
        galileo_status.targetStatus = 0
        if monitor_server.nav_task != None:
            if monitor_server.nav_task.current_goal_status() == "FREE":
                galileo_status.targetStatus = 0
            if monitor_server.nav_task.current_goal_status() == "WORKING":
                galileo_status.targetStatus = 1
            if monitor_server.nav_task.current_goal_status() == "PAUSED":
                galileo_status.targetStatus = 2
            if monitor_server.nav_task.current_goal_status() == "ERROR":
                galileo_status.targetStatus = -1
        galileo_status.targetDistance = -1
        if monitor_server.nav_task != None and \
                monitor_server.nav_task.current_goal_status() != "ERROR":
            galileo_status.targetDistance = \
                monitor_server.nav_task.current_goal_distance()
        galileo_status.angleGoalStatus = 1
        if ROBOT_CONTROL_TWIST is not None and ROBOT_REAL_TWIST is not None:
            galileo_status.controlSpeedX = ROBOT_CONTROL_TWIST.linear.x
            galileo_status.controlSpeedTheta = ROBOT_CONTROL_TWIST.angular.z
            galileo_status.currentSpeedX = ROBOT_REAL_TWIST.twist.linear.x
            galileo_status.currentSpeedTheta = ROBOT_REAL_TWIST.twist.angular.z
        if monitor_server.map_thread.stopped():
            galileo_status.mapStatus = 0
        else:
            galileo_status.mapStatus = 1
        if ROBOT_STATUS.orbGCFlag:
            galileo_status.gcStatus = 1
        else:
            galileo_status.gcStatus = 0
        if ROBOT_STATUS.orbGBAFlag:
            galileo_status.gbaStatus = 1
        else:
            galileo_status.gbaStatus = 0
        galileo_status.chargeStatus = CHARGE_STATUS.data
        if galileo_status.header.stamp == rospy.Time(0):
            galileo_status.header.stamp = rospy.Time.now()

        pubs["GALILEO_STATUS_PUB"].publish(galileo_status)

        # 每秒广播一次
        if broadcast_count == 10:
            broadcast_count = 0
            data = "xq"
            # 发送广播包
            try:
                s.sendto(data, ('<broadcast>', BROADCAST_PORT))
            except:
                continue
            ROBOT_STATUS.brightness = 0.0
            ROBOT_STATUS.imageStatus = False
            ROBOT_STATUS.odomStatus = False
            ROBOT_STATUS.orbStartStatus = False
            ROBOT_STATUS.orbInitStatus = False
            ROBOT_STATUS.power = 0.0
            ROBOT_STATUS.orbScaleStatus = False
        broadcast_count += 1
        rate.sleep()
    monitor_server.stop()
