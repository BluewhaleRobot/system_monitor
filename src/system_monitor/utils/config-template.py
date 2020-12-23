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

import numpy as np
import rospy

HOST = ''  # should not be 127.0.0.1 or localhost
USERSOCKET_PORT = 20001  # 局域网udp命令监听端口
BROADCAST_PORT = 22001 # 局域网广播端口
BROADCAST_PORT_V2 = 22002 # 局域网广播端口V2版本

MAX_VEL = 0.8
MAX_THETA = 1.2
POWER_LOW = 22.5

TF_ROT = np.array([[0., 0., 1.],
                   [-1., 0., 0.], [0., -1., 0.]])
TF_TRANS = np.array([0.05, 0.0, 0.])
ROS_PACKAGE_PATH = '/home/xiaoqiang/Documents/ros/src:/opt/ros/kinetic/share:' + \
    '/opt/ros/kinetic/stacks:' + \
    '/home/xiaoqiang/Documents/ros/src/ORB_SLAM2/Examples/ROS'
SHARPLINK_LOG_FILE = "/home/xiaoqiang/Documents/ros/devel/lib/sharplink/server.log"
IOT_SECRET = "Jbk3GLKIZYIRrTgthbAEa9TlXHdae5UL"
IOT_KEY = "a1Eb29fVWHG"
IOT_PASSWORD = "xiaoqiang"
ALLOW_LOCAL_ONLY = False
