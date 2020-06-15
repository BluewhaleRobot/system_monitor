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

'''
This is a system monitor node for xiaoqiang. Monitor items are power, odom, brightness etc.
System status will be published at /system_monitor/report
'''

import json
import threading

import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64, UInt32, Int32
from system_monitor.msg import Status
from xiaoqiang_log.msg import LogRecord

from utils.config import POWER_LOW

REPORT_PUB = ""
ROBOT_STATUS = Status()
ROBOT_STATUS.brightness = 0
ROBOT_STATUS.imageStatus = False
ROBOT_STATUS.odomStatus = False
ROBOT_STATUS.orbStartStatus = False
ROBOT_STATUS.orbInitStatus = False
ROBOT_STATUS.power = 0
ROBOT_STATUS.orbScaleStatus = False


STATUS_LOCK = threading.Lock()
POWER_LAST = 0


def get_brightness(brightness):
    with STATUS_LOCK:
        ROBOT_STATUS.brightness = brightness.data


def get_image(image):
    with STATUS_LOCK:
        if image != None:
            ROBOT_STATUS.imageStatus = True
        else:
            ROBOT_STATUS.imageStatus = False


def get_power(power):
    global POWER_LAST

    with STATUS_LOCK:
        if ROBOT_STATUS.power < 0.1:
            POWER_LAST = power.data
        if POWER_LAST < power.data + 0.7 and POWER_LAST > power.data - 0.7:
            ROBOT_STATUS.power = power.data
        POWER_LAST = power.data


def get_odom(odom):
    with STATUS_LOCK:
        if odom != None:
            ROBOT_STATUS.odomStatus = True
        else:
            ROBOT_STATUS.odomStatus = False


def get_orb_start_status(orb_frame):
    with STATUS_LOCK:
        if orb_frame != None:
            ROBOT_STATUS.orbStartStatus = True
        else:
            ROBOT_STATUS.orbStartStatus = False


def get_orb_track_flag(cam_pose):
    with STATUS_LOCK:
        if cam_pose != None:
            ROBOT_STATUS.orbInitStatus = True
        else:
            ROBOT_STATUS.orbInitStatus = False


def get_orb_scale_status(flag):
    with STATUS_LOCK:
        ROBOT_STATUS.orbScaleStatus = flag.data


def monitor():
    global REPORT_PUB
    rospy.init_node("monitor", anonymous=True)
    rospy.Subscriber("/usb_cam/brightness", UInt32, get_brightness)
    rospy.Subscriber("/usb_cam/camera_info", rospy.AnyMsg, get_image)
    rospy.Subscriber("/ORB_SLAM/TrackingStatus", Int32, get_orb_start_status)
    rospy.Subscriber("/xqserial_server/Power", Float64, get_power)
    rospy.Subscriber("/xqserial_server/Odom", Odometry, get_odom)
    rospy.Subscriber("/ORB_SLAM/Camera", Pose, get_orb_track_flag)
    rospy.Subscriber("/orb_scale/scaleStatus", Bool, get_orb_scale_status)
    REPORT_PUB = rospy.Publisher(
        '/system_monitor/report', Status, queue_size=0)


if __name__ == "__main__":
    monitor()
    rate = rospy.Rate(1)
    log_count = 0
    log_pub = rospy.Publisher("/xiaoqiang_log", LogRecord, queue_size=0)

    while not rospy.is_shutdown():
        STATUS_LOCK.acquire()
        log_count += 1
        # 每1min记录一次电压
        if log_count == 60:
            log_count = 1
            log_record = LogRecord()
            log_record.stamp = rospy.Time.now()
            log_record.collection_name = "power"
            log_record.record = json.dumps({
                "power": ROBOT_STATUS.power
            }, indent=4)
            log_pub.publish(log_record)
        if REPORT_PUB != "":
            REPORT_PUB.publish(ROBOT_STATUS)
        # clear data
        ROBOT_STATUS.brightness = 0
        ROBOT_STATUS.power = 0
        ROBOT_STATUS.imageStatus = False
        ROBOT_STATUS.odomStatus = False
        ROBOT_STATUS.orbInitStatus = False
        ROBOT_STATUS.orbStartStatus = False
        ROBOT_STATUS.orbScaleStatus = False
        STATUS_LOCK.release()
        rate.sleep()
