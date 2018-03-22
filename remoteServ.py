#!/usr/bin/env python
#coding:utf-8

import rospy
from std_msgs.msg import String, UInt32, Float64, Bool,Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D, Pose,PoseStamped
from sensor_msgs.msg import Image
from system_monitor.msg import *
import threading
import os
import sys
from socket import *
import commands
import struct
from geometry_msgs.msg import Twist
import time,psutil,subprocess,signal
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import numpy as  np
from utils.req_parser import ReqParser
from utils.monitor_servers import UserSer
from utils.config import BROADCAST_PORT
from galileo_serial_server.msg import GalileoNativeCmds, GalileoStatus

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

SEND_DATA = bytearray(
[205, 235, 215, 32, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00])

CONTROL_FLAG = False

TF_ROT = np.array([[ 0., 0.03818382, 0.99927073],[ -1., 0.,0.], [0., -0.99927073, 0.03818382]])
TF_TRANS = np.array([0.4,0.0,0.])
TILT_PUB = None
NAV_LASTTIME = None

Thf=[0.0,0.0,0.0]
Qhf=[0.0,0.0,0.0,1.0]

def get_power(power):
    ROBOT_STATUS_LOCK.acquire()
    ROBOT_STATUS.power = power.data
    #if power.data < POWER_LOW and not os.path.isfile(powerFlagFilePath) and power.data > 0.1:
    ROBOT_STATUS_LOCK.release()

def get_image(image):
    ROBOT_STATUS_LOCK.acquire()
    if image != None:
        ROBOT_STATUS.imageStatus = True
    else:
        ROBOT_STATUS.imageStatus = False
    ROBOT_STATUS_LOCK.release()

def get_odom(odom):
    global ROBOT_POSESTAMPED, ROBOT_REAL_TWIST, ROBOT_STATUS
    ROBOT_STATUS_LOCK.acquire()
    if odom != None:
        ROBOT_STATUS.odomStatus = True
        ROBOT_POSESTAMPED = PoseStamped()
        ROBOT_POSESTAMPED.pose=odom.pose.pose; #更新坐标
        ROBOT_POSESTAMPED.header=odom.header #更新坐标
        ROBOT_REAL_TWIST = odom.twist
    else:
        ROBOT_STATUS.odomStatus = False
    ROBOT_STATUS_LOCK.release()

def get_cmd_vel(twist):
    global ROBOT_CONTROL_TWIST
    if twist != None:
        ROBOT_CONTROL_TWIST = twist

def get_orb_start_status(orb_frame):
    ROBOT_STATUS_LOCK.acquire()
    if orb_frame != None:
        ROBOT_STATUS.orbStartStatus = True
    else:
        ROBOT_STATUS.orbStartStatus = False
    ROBOT_STATUS_LOCK.release()

def get_orb_tracking_flag(cam_pose):
    ROBOT_STATUS_LOCK.acquire()
    if cam_pose != None:
        ROBOT_STATUS.orbInitStatus = True
    else:
        ROBOT_STATUS.orbInitStatus = False
    ROBOT_STATUS_LOCK.release()

def get_global_move_flag(moveEn):
    if not moveEn.data:
        #关闭视觉导航
        if not UserServer.NAV_THREAD.stopped():
            UserServer.NAV_THREAD.stop()
            UserServer.NAV_FLAG=False

def get_nav_flag(navRun):
    if navRun.data and not UserServer.NAV_THREAD.stopped():
        UserServer.NAV_FLAG=True
    else:
        UserServer.NAV_FLAG=False

def get_orbgc_status(gc_flag):
    ROBOT_STATUS_LOCK.acquire()
    ROBOT_STATUS.orbGCFlag = gc_flag.data
    ROBOT_STATUS_LOCK.release()

def get_orbgba_status(gba_flag):
    ROBOT_STATUS_LOCK.acquire()
    ROBOT_STATUS.orbGBAFlag = gba_flag.data
    ROBOT_STATUS_LOCK.release()


def init_sub_pubs():
    rospy.init_node("broadcast", anonymous=True)
    NAV_LASTTIME = rospy.Time.now()
    rospy.Subscriber("/xqserial_server/Power", Float64, get_power)
    rospy.Subscriber("/usb_cam/image_raw", Image, get_image)
    rospy.Subscriber("/xqserial_server/Odom", Odometry, get_odom)
    rospy.Subscriber("/ORB_SLAM/Camera", Pose, get_orb_tracking_flag)
    rospy.Subscriber("/ORB_SLAM/Frame", Image, get_orb_start_status)
    rospy.Subscriber("/ORB_SLAM/GC", Bool, get_orbgc_status)
    rospy.Subscriber("/ORB_SLAM/GBA", Bool, get_orbgba_status)
    rospy.Subscriber("/global_move_flag", Bool, get_global_move_flag)
    rospy.Subscriber('/nav_setStop', Bool, get_nav_flag)
    rospy.Subscriber('/cmd_vel', Twist, get_cmd_vel)
    GLOBAL_MOVE_PUB = rospy.Publisher('/global_move_flag', Bool , queue_size=1)
    ELEVATOR_PUB = rospy.Publisher('/elevatorPose', UInt32 , queue_size=1)
    CMD_VEL_PUB = rospy.Publisher('/cmd_vel', Twist , queue_size=0)
    MAPSAVE_PUB = rospy.Publisher('/map_save', Bool , queue_size=0)
    TILT_PUB = rospy.Publisher('/set_tilt_degree', Int16 , queue_size=0)
    GALILEO_STATUS_PUB = rospy.Publisher('/galileo/status', GalileoStatus, queue_size=0)
    return {
        "GLOBAL_MOVE_PUB": GLOBAL_MOVE_PUB,
        "ELEVATOR_PUB": ELEVATOR_PUB,
        "CMD_VEL_PUB": CMD_VEL_PUB,
        "MAPSAVE_PUB": MAPSAVE_PUB,
        "TILT_PUB": TILT_PUB,
        "GALILEO_STATUS_PUB": GALILEO_STATUS_PUB,
    }

if __name__ == "__main__":
    pubs = init_sub_pubs()
    rate = rospy.Rate(10)
    #配置udp广播
    s = socket(AF_INET, SOCK_DGRAM)
    s.bind(('', 0))
    s.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)
    #开启udp接收监听线程
    UserServer = UserSer(pubs, ROBOT_STATUS_LOCK, ROBOT_STATUS)
    UserServer.start()
    broadcast_count = 10 #每1秒播放一次声音
    heart_beat_count = 40 #每4秒心跳维护一次
    play_sound_cmd = "aplay /home/xiaoqiang/Desktop/d.wav"
    listener = tf.TransformListener(True, rospy.Duration(10.0))
    while not rospy.is_shutdown():
        # #每２秒提示一下
        # if heart_beat_count==10:
        #     subprocess.Popen(play_sound_cmd,shell=True)

        # if not CONTROL_FLAG and heart_beat_count>32 and  heart_beat_count<37:
        #     SPEED_CMD.linear.x = 0
        #     SPEED_CMD.angular.z = 0
        #     CMD_VEL_PUB.publish(SPEED_CMD)

        if heart_beat_count == 40:
            heart_beat_count = 0
            CONTROL_FLAG = False
        heart_beat_count += 1
        #持续反馈状态
        if UserServer.get_connection_status() and ROBOT_POSESTAMPED is not None:
            tfFlag = False
            try:
                ROBOT_POSESTAMPED = listener.transformPose("/map", ROBOT_POSESTAMPED)
                tfFlag = True
                (Thf, Qhf) = listener.lookupTransform("/map", "/odom", ROBOT_POSESTAMPED.header.stamp)
            except (tf.LookupException,tf.ConnectivityException,tf.ExtrapolationException,tf.Exception):
                tfFlag=False

            #将机器人坐标系转换成map坐标系
            currentPose = ROBOT_POSESTAMPED.pose
            Tbc = np.array([currentPose.position.x, currentPose.position.y, currentPose.position.z])
            q = [currentPose.orientation.x, currentPose.orientation.y, currentPose.orientation.z, currentPose.orientation.w]
            M = tf.transformations.quaternion_matrix(q)
            Rbc = M[:3, :3]
            if not tfFlag:
                M = tf.transformations.quaternion_matrix(Qhf)
                Rhf = M[:3,:3]
                Rbc = Rhf.dot(Rbc)
                Tbc = Rhf.dot(Tbc) + Thf

            ax,ay,theta_send = tf.transformations.euler_from_matrix(Rbc)
            #为了简化计算，下文的计算中 base_link 和 base_footprint 被看成是相同的坐标系
            #Rbd = Rbc.dot(TF_ROT)
            #Tbd = Rbc.dot(TF_TRANS) + Tbc

            Rab = TF_ROT.T
            Tab = -Rab.dot(TF_TRANS)

            #Rad = Rab.dot(Rbd)
            #Tad = Rab.dot(Tbd)+Tab

            Rad=Rab.dot(Rbc)
            Tad=Rab.dot(Tbc)+Tab

            SEND_DATA[4:8] = map(ord,struct.pack('f',Tad[0]))
            SEND_DATA[8:12] = map(ord,struct.pack('f',Tad[1]))
            SEND_DATA[12:16] = map(ord,struct.pack('f',Tad[2]))
            SEND_DATA[16:20] = map(ord,struct.pack('f',ROBOT_STATUS.power))
            SEND_DATA[24:28] = map(ord,struct.pack('f',theta_send))
            if UserServer.nav_task == None:
                SEND_DATA[28:32] = map(ord, struct.pack('i', 3))
                SEND_DATA[32:36] = map(ord, struct.pack('i', -1))
            else:
                if UserServer.nav_task.current_goal_status() == "FREE":
                    SEND_DATA[28:32] = map(ord, struct.pack('i', 0))
                    SEND_DATA[32:36] = map(ord, struct.pack('i', -1))
                if UserServer.nav_task.current_goal_status() == "WORKING":
                    SEND_DATA[28:32] = map(ord, struct.pack('i', 1))
                    SEND_DATA[32:36] = map(ord, struct.pack('i',
                        UserServer.nav_task.current_goal_id))
                if UserServer.nav_task.current_goal_status() == "PAUSED":
                    SEND_DATA[28:32] = map(ord, struct.pack('i', 2))
                    SEND_DATA[32:36] = map(ord, struct.pack('i',
                        UserServer.nav_task.current_goal_id))

            if tfFlag or UserServer.NAV_FLAG or not UserServer.NAV_THREAD.stopped():
                statu0=0x01 #混合里程计
            else:
                statu0=0x00
            if ROBOT_STATUS.imageStatus :
                statu1=0x02 #视觉摄像头
            else:
                statu1=0x00
            if ROBOT_STATUS.orbStartStatus :
                statu2=0x04 #视觉系统状态
            else:
                statu2=0x00
            if ROBOT_STATUS.orbInitStatus :
                statu3=0x08 #视觉系统状态
            else:
                statu3=0x00
            if ROBOT_STATUS.orbGCFlag:
                status4 = 0x10 #ORB_SLAM 内存回收状态
            else:
                status4 = 0
            if ROBOT_STATUS.orbGBAFlag:
                status5 = 0x20 #ORB_SLAM2 GBA状态，一般对应LoopClosing
            else:
                status5 = 0x00
            SEND_DATA[20] = statu0 + statu1 + statu2 + statu3 + status4 + status5
            UserServer.sendto(bytes(SEND_DATA))

        #每秒广播一次
        if broadcast_count == 10:
            broadcast_count = 0;
            data = "xq"
            #发送广播包
            try:
                s.sendto(data, ('<broadcast>', BROADCAST_PORT))
            except:
                continue
            # clear data
            ROBOT_STATUS.power = 0.0
            ROBOT_STATUS.orbInitStatus = False
            ROBOT_STATUS.orbStartStatus = False
            ROBOT_STATUS.imageStatus = False
            ROBOT_STATUS.odomStatus = False
            ROBOT_STATUS.orbGCFlag = False
            ROBOT_STATUS.orbGBAFlag = False

        # 发布状态topic
        galileo_status = GalileoStatus()
        galileo_status.header = ROBOT_POSESTAMPED.header
        galileo_status.navStatus = 0
        if ROBOT_STATUS.orbInitStatus:
            galileo_status.visualStatus = 1
        else:
            galileo_status.visualStatus = 2
        if UserServer.nav_task != None:
            galileo_status.navStatus = 1
        else:
            galileo_status.visualStatus = 0
        galileo_status.power = ROBOT_STATUS.power
        galileo_status.targetNumID = -1
        if UserServer.nav_task != None:
            galileo_status.targetNumID = UserServer.nav_task.current_goal_id
        galileo_status.targetStatus = 0
        if UserServer.nav_task != None:
            if UserServer.nav_task.current_goal_status() == "FREE":
                galileo_status.targetStatus = 0
            if UserServer.nav_task.current_goal_status() == "WORKING":
                galileo_status.targetStatus = 1
            if UserServer.nav_task.current_goal_status() == "PAUSED":
                galileo_status.targetStatus = 2
            if UserServer.nav_task.current_goal_status() == "ERROR":
                galileo_status.targetStatus = -1
        galileo_status.targetDistance = -1
        if UserServer.nav_task != None and \
            UserServer.nav_task.current_goal_status() != "ERROR":
            galileo_status.targetDistance = \
                UserServer.nav_task.current_goal_distance()
        galileo_status.angleGoalStatus = 1
        if ROBOT_CONTROL_TWIST is not None and ROBOT_REAL_TWIST is not None:
            galileo_status.controlSpeedX = ROBOT_CONTROL_TWIST.linear.x
            galileo_status.controlSpeedTheta = ROBOT_CONTROL_TWIST.angular.z
            galileo_status.currentSpeedX = ROBOT_REAL_TWIST.twist.linear.x
            galileo_status.currentSpeedTheta = ROBOT_REAL_TWIST.twist.angular.z
        pubs["GALILEO_STATUS_PUB"].publish(galileo_status)

        broadcast_count += 1;
        rate.sleep()
    UserServer.stop()
