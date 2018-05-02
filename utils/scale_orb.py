#!/usr/bin/env python
# coding:utf-8

import rospy
from std_msgs.msg import String, UInt32, Float64, Bool, Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D, Pose, PoseStamped
from sensor_msgs.msg import Image
from system_monitor.msg import *
import threading
import os
import sys
from socket import *
import commands
import struct
from geometry_msgs.msg import Twist
import time
import psutil
import subprocess
import signal
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import numpy as np


class ScaleORB(threading.Thread):

    def __init__(self, ROBOT_STATUS):
        super(ScaleORB, self).__init__()
        self._stop = threading.Event()
        self.car_odoms = 0.0
        self.cam_odoms = 0.0
        self.car_lastPose = None
        self.cam_lastPose = None
        self.scale = 1.
        self.carPoseLock = threading.Lock()
        self.camPoseLock = threading.Lock()
        self.scaleLock = threading.Lock()
        self.carbegin_flag = True
        self.cambegin_flag = True
        self.ROBOT_STATUS = ROBOT_STATUS

    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()

    def run(self):

        def updateCar(pose2d):
            self.carPoseLock.acquire()
            self.scaleLock.acquire()
            if self.carbegin_flag:
                self.car_lastPose = pose2d
                self.carbegin_flag = False
            self.car_odoms += math.sqrt((pose2d.x - self.car_lastPose.x) * (pose2d.x - self.car_lastPose.x) + (
                pose2d.y - self.car_lastPose.y) * (pose2d.y - self.car_lastPose.y))
            self.scaleLock.release()
            self.car_lastPose = pose2d
            self.carPoseLock.release()

        def updateCam(pose):
            self.camPoseLock.acquire()
            self.scaleLock.acquire()
            if self.cambegin_flag:
                self.cam_lastPose = pose
                self.cambegin_flag = False
            self.cam_odoms += math.sqrt((pose.position.x - self.cam_lastPose.position.x) * (pose.position.x - self.cam_lastPose.position.x) + (
                pose.position.z - self.cam_lastPose.position.z) * (pose.position.z - self.cam_lastPose.position.z))
            self.scaleLock.release()
            self.cam_lastPose = pose
            self.camPoseLock.release()
        while not self.ROBOT_STATUS.orbInitStatus and not rospy.is_shutdown():
            time.sleep(0.5)
        if rospy.is_shutdown():
            self.stop()
            return
        carSub = rospy.Subscriber("/xqserial_server/Pose2D", Pose2D, updateCar)
        camSub = rospy.Subscriber("/ORB_SLAM/Camera", Pose, updateCam)
        while not self.stopped() and not rospy.is_shutdown():
            time.sleep(1)
        carSub.unregister()
        camSub.unregister()
        self.stop()

    def saveScale(self):
        self.scaleLock.acquire()
        if self.cam_odoms > 0.01:
            self.scale = self.car_odoms / self.cam_odoms
        else:
            self.scale = 1.0
        fp3 = open("/home/xiaoqiang/slamdb/scale.txt", 'a+')
        # +" "+str(self.car_odoms)+" "+str(self.cam_odoms))
        fp3.write(str(self.scale))
        fp3.write('\n')
        fp3.close
        self.scaleLock.release()
