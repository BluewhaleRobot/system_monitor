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

class ReqParser(object):
    """docstring for ReqParser
       解析自定义数据包的类
    """
    def __init__(self, PACKAGE_HEADER=[205, 235, 215], BUFSIZE = 1024):
        super(ReqParser, self).__init__()
        self.DATA_CACHE = []
        self.PACKAGE_HEADER = PACKAGE_HEADER

    def unpack_req(self, req):
        res = []
        package_list = self.split_req(req)
        # process the first package
        complete_data = self.DATA_CACHE + package_list[0]
        package_list.remove(package_list[0])
        package_list =  self.split_req(complete_data) + package_list

        for count in range(0, len(package_list)):
            if len(package_list[count]) != 0 and \
            len(package_list[count]) == package_list[count][0] + 1:
                res.append(package_list[count][1:])
        last_one = package_list[-1:][0] # the last one
        if len(last_one) == 0 or len(last_one) != last_one[0] + 1:
            self.DATA_CACHE = last_one
        return res


    def find_package_header(self, req):
        if len(req) < 3:
            return -1
        for count in range(0, len(req) - 2):
            if req[count] == self.PACKAGE_HEADER[0] and \
            req[count + 1] == self.PACKAGE_HEADER[1] and \
            req[count + 2] == self.PACKAGE_HEADER[2]:
                return count
        return -1

    def split_req(self, req):
        res = []
        start_index = 0
        new_index = 0
        while True:
            new_index = self.find_package_header(req[start_index:])
            if new_index == -1:
                break
            res.append(req[start_index: start_index + new_index])
            start_index = new_index + 3 + start_index
        res.append(req[start_index:])
        return res
