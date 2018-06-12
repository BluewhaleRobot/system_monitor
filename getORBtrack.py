#!/usr/bin/env python
# coding:utf-8
'''
使orb获得初始追踪，发布kinect角度，监听系统状态
'''

import rospy
from std_msgs.msg import String, UInt32, Float64, Bool, Int32, Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D, Pose
from sensor_msgs.msg import Image
from system_monitor.msg import *
from system_monitor.msg import Status
import math
import threading
import os
import time
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import scipy.linalg

getTrackThread = None
orbInitFlag = False
orbStartFlag = False
mStatusLock = threading.Lock()

velPub = None


globalMovePub = None

camUpdateFlag = False
carUpdateFlag = False

cam_currentTime = None
car_currentTime = None

barFlag = False
navFlag_pub = None

enableMoveFlag = True
# try to get track again when losing track


import threading


class getTrack(threading.Thread):

    def __init__(self):
        super(getTrack, self).__init__()
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()

    def run(self):
        global velPub, getTrackThread
        if velPub == "":
            # driver is not ready
            getTrackThread = None
            return
        print "start get track"
        CarTwist = Twist()
        # rotate small angels
        CarTwist.linear.x = 0.0
        CarTwist.linear.y = 0.0
        CarTwist.linear.z = 0.0
        CarTwist.angular.x = 0.0
        CarTwist.angular.y = 0.0
        CarTwist.angular.z = 0.3
        velPub.publish(CarTwist)
        self.wait(3)
        CarTwist.angular.z = -0.3
        velPub.publish(CarTwist)
        self.wait(8)
        CarTwist.angular.z = 0.
        velPub.publish(CarTwist)
        getTrackThread = None
        print "stop get track"

    def wait(self, waitTime):
        sleepTime = 0
        while sleepTime < waitTime and not self.stopped():
            time.sleep(0.05)
            sleepTime += 0.05


def systemStatusHandler(system_status):
    global orbInitFlag, mStatusLock, orbStartFlag
    mStatusLock.acquire()
    orbInitFlag = system_status.orbInitStatus
    orbStartFlag = system_status.orbStartStatus
    mStatusLock.release()


def carOdom(odom):
    global car_currentPose, car_currentTime, mStatusLock, begin_flag, carUpdateFlag
    mStatusLock.acquire()
    car_currentTime = rospy.Time.now()
    carUpdateFlag = True
    mStatusLock.release()


def cameraOdom(campose):
    global cam_currentPose, cam_currentTime, mStatusLock, begin_flag, camUpdateFlag
    mStatusLock.acquire()
    cam_currentTime = rospy.Time.now()
    camUpdateFlag = True
    mStatusLock.release()


def doSecurity():
    global mStatusLock,carUpdateFlag,camUpdateFlag
    global cam_currentTime, car_currentTime
    global globalMovePub, velPub
    global navFlag_pub, enableMoveFlag,barFlag

    time_now = rospy.Time.now()

    mStatusLock.acquire()

    if carUpdateFlag:
        time1_diff = time_now - car_currentTime

    if camUpdateFlag:
        time2_diff = time_now - cam_currentTime

    #视觉丢失超时保险
    #里程计丢失超时保险
    if (camUpdateFlag and time2_diff.to_sec() > 180 and not barFlag) or time1_diff.to_sec() > 5.:
        # 发布navFlag
        if navFlag_pub != None:
            navFlag = Bool()
            navFlag.data = True
            navFlag_pub.publish(navFlag)
        # 发布全局禁止ｆｌａｇ
        globalMoveFlag = Bool()
        globalMoveFlag.data = False
        globalMovePub.publish(globalMoveFlag)
        CarTwist = Twist()
        velPub.publish(CarTwist)
        camUpdateFlag = False
        carUpdateFlag = False
        cam_currentTime = rospy.Time.now()
        car_currentTime = rospy.Time.now()
    mStatusLock.release()


def dealCarStatus(carStatu):
    global barFlag
    status = carStatu.data
    if status == 2:
        barFlag = True
    else:
        barFlag = False


def init():
    global orbInitFlag, orbStartFlag
    global velPub, getTrackThread, globalMovePub
    global cam_currentTime, car_currentTime
    global  navFlag_pub, enableMoveFlag
    barFlag = False

    rospy.init_node("getORRtrack", anonymous=True)

    cam_currentTime = rospy.Time.now()
    car_currentTime = rospy.Time.now()

    enableMoveFlag = rospy.get_param("~enableMoveFlag", True)

    rospy.Subscriber("/system_monitor/report", Status, systemStatusHandler)
    rospy.Subscriber("/ORB_SLAM/Camera", Pose, cameraOdom)
    rospy.Subscriber("/xqserial_server/Odom", Odometry, carOdom)
    rospy.Subscriber("/xqserial_server/StatusFlag", Int32, dealCarStatus)
    globalMovePub = rospy.Publisher('/globalMoveFlag', Bool, queue_size=1)
    navFlag_pub = rospy.Publisher('/nav_setStop', Bool, queue_size=0)
    velPub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

    # 确保orb 已经启动
    while not orbStartFlag and not rospy.is_shutdown():
        print "oups! 1"
        time.sleep(1)
    # 确保orb　已经 track
    time.sleep(4)
    print "oups! 2"
    globalMoveFlag = Bool()
    globalMoveFlag.data = True
    globalMovePub.publish(globalMoveFlag)

    while not orbInitFlag and not rospy.is_shutdown() and enableMoveFlag:
        if getTrackThread == None:
            getTrackThread = getTrack()
            getTrackThread.start()
            time.sleep(6)
        time.sleep(1)
    if getTrackThread != None:
        getTrackThread.stop()


if __name__ == "__main__":
    init()
    rate = rospy.Rate(10)
    tilt_pub = rospy.Publisher('/set_tilt_degree', Int16, queue_size=1)
    while not rospy.is_shutdown():
        doSecurity()
        tilt_degree = Int16()
        tilt_degree.data = -19
        tilt_pub.publish(tilt_degree)
        rate.sleep()
    if getTrackThread != None:
        getTrackThread.stop()
