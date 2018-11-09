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


'''
使orb获得初始追踪，发布kinect角度，监听系统状态
'''

import threading
import time

import rospy
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int16, Int32
from system_monitor.msg import Status

ORB_TRACK_THREAD = None
ORB_INIT_FLAG = False
ORB_START_FLAG = False
STATUS_LOCK = threading.Lock()

VEL_PUB = None


GLOBAL_MOVE_PUB = None

CAMERA_UPDATE_FLAG = False
CAR_UPDATE_FLAG = False

CAMERA_CURRENT_TIME = None
CAR_CURRENT_TIME = None

BAR_FLAG = False
NAV_FLAG_PUB = None

ENABLE_MOVE_FLAG = True


class TrackTask(threading.Thread):

    def __init__(self):
        super(TrackTask, self).__init__()
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()

    def run(self):
        global VEL_PUB, ORB_TRACK_THREAD
        if VEL_PUB == "":
            # driver is not ready
            ORB_TRACK_THREAD = None
            return
        rospy.loginfo("start get track")
        car_twist = Twist()
        # rotate small angels
        car_twist.linear.x = 0.0
        car_twist.linear.y = 0.0
        car_twist.linear.z = 0.0
        car_twist.angular.x = 0.0
        car_twist.angular.y = 0.0
        car_twist.angular.z = 0.3
        VEL_PUB.publish(car_twist)
        self.wait(3)
        car_twist.angular.z = -0.3
        VEL_PUB.publish(car_twist)
        self.wait(8)
        car_twist.angular.z = 0.
        VEL_PUB.publish(car_twist)
        ORB_TRACK_THREAD = None
        rospy.loginfo("stop get track")

    def wait(self, wait_time):
        sleep_time = 0
        while sleep_time < wait_time and not self.stopped():
            time.sleep(0.05)
            sleep_time += 0.05


def system_status_handler(system_status):
    global ORB_INIT_FLAG, STATUS_LOCK, ORB_START_FLAG
    with STATUS_LOCK:
        ORB_INIT_FLAG = system_status.orbInitStatus
        ORB_START_FLAG = system_status.orbStartStatus


def car_odom(odom):
    global CAR_CURRENT_TIME, STATUS_LOCK, CAR_UPDATE_FLAG
    with STATUS_LOCK:
        CAR_CURRENT_TIME = rospy.Time.now()
        CAR_UPDATE_FLAG = True


def camera_odom(campose):
    global CAMERA_CURRENT_TIME, STATUS_LOCK, CAMERA_UPDATE_FLAG
    with STATUS_LOCK:
        CAMERA_CURRENT_TIME = rospy.Time.now()
        CAMERA_UPDATE_FLAG = True


def do_security():
    global STATUS_LOCK, CAR_UPDATE_FLAG, CAMERA_UPDATE_FLAG
    global CAMERA_CURRENT_TIME, CAR_CURRENT_TIME
    global GLOBAL_MOVE_PUB, VEL_PUB
    global NAV_FLAG_PUB, ENABLE_MOVE_FLAG, BAR_FLAG

    time_now = rospy.Time.now()

    STATUS_LOCK.acquire()

    if CAR_UPDATE_FLAG:
        time1_diff = time_now - CAR_CURRENT_TIME

    if CAMERA_UPDATE_FLAG:
        time2_diff = time_now - CAMERA_CURRENT_TIME

    # 视觉丢失超时保险
    # 里程计丢失超时保险
    if (CAMERA_UPDATE_FLAG and time2_diff.to_sec() > 180 and not BAR_FLAG) or time1_diff.to_sec() > 5.:
        # 发布navFlag
        if NAV_FLAG_PUB != None:
            nav_flag = Bool()
            nav_flag.data = True
            NAV_FLAG_PUB.publish(nav_flag)
        # 发布全局禁止ｆｌａｇ
        global_move_flag = Bool()
        global_move_flag.data = False
        GLOBAL_MOVE_PUB.publish(global_move_flag)
        car_twist = Twist()
        VEL_PUB.publish(car_twist)
        CAMERA_UPDATE_FLAG = False
        CAR_UPDATE_FLAG = False
        CAMERA_CURRENT_TIME = rospy.Time.now()
        CAR_CURRENT_TIME = rospy.Time.now()
    STATUS_LOCK.release()


def deal_car_status(car_status):
    global BAR_FLAG
    status = car_status.data
    if status == 2:
        BAR_FLAG = True
    else:
        BAR_FLAG = False


def init():
    global ORB_INIT_FLAG, ORB_START_FLAG
    global VEL_PUB, ORB_TRACK_THREAD, GLOBAL_MOVE_PUB
    global CAMERA_CURRENT_TIME, CAR_CURRENT_TIME
    global NAV_FLAG_PUB, ENABLE_MOVE_FLAG

    rospy.init_node("orb_track", anonymous=True)

    CAMERA_CURRENT_TIME = rospy.Time.now()
    CAR_CURRENT_TIME = rospy.Time.now()

    ENABLE_MOVE_FLAG = rospy.get_param("~enableMoveFlag", True)

    rospy.Subscriber("/system_monitor/report", Status, system_status_handler)
    rospy.Subscriber("/ORB_SLAM/Camera", Pose, camera_odom)
    rospy.Subscriber("/xqserial_server/Odom", Odometry, car_odom)
    rospy.Subscriber("/xqserial_server/StatusFlag", Int32, deal_car_status)
    GLOBAL_MOVE_PUB = rospy.Publisher('/global_move_flag', Bool, queue_size=1)
    NAV_FLAG_PUB = rospy.Publisher('/nav_setStop', Bool, queue_size=0)
    VEL_PUB = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

    # 确保orb 已经启动
    while not ORB_START_FLAG and not rospy.is_shutdown():
        rospy.logwarn("ORB_SLAM2 not started")
        time.sleep(1)
    # 确保orb　已经 track
    time.sleep(4)
    rospy.logwarn("ORB_SLAM2 not track")
    globalMoveFlag = Bool()
    globalMoveFlag.data = True
    GLOBAL_MOVE_PUB.publish(globalMoveFlag)

    while not ORB_INIT_FLAG and not rospy.is_shutdown() and ENABLE_MOVE_FLAG:
        if ORB_TRACK_THREAD == None:
            ORB_TRACK_THREAD = TrackTask()
            ORB_TRACK_THREAD.start()
            time.sleep(6)
        time.sleep(1)
    if ORB_TRACK_THREAD != None:
        ORB_TRACK_THREAD.stop()


if __name__ == "__main__":
    init()
    rate = rospy.Rate(10)
    tilt_pub = rospy.Publisher('/set_tilt_degree', Int16, queue_size=1)
    while not rospy.is_shutdown():
        do_security()
        tilt_degree = Int16()
        tilt_degree.data = -19
        tilt_pub.publish(tilt_degree)
        rate.sleep()
    if ORB_TRACK_THREAD != None:
        ORB_TRACK_THREAD.stop()
