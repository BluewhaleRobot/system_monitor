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
# Author: Randoms
#

import struct
import threading

import numpy as np
import rospy
import tf
from galileo_serial_server.msg import GalileoStatus
from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav_msgs.msg import Odometry
from bw_env_sensors.msg import EnvSensors

from config import BROADCAST_PORT, ROS_PACKAGE_PATH, TF_ROT, TF_TRANS


class UDPStatusService(threading.Thread):

    def __init__(self, monitor_server, galileo_status, galileo_status_lock, useEnvSensors):
        super(UDPStatusService, self).__init__()
        # stop flag
        self._stop = threading.Event()
        self._stop.set()
        self.listener = tf.TransformListener(True, rospy.Duration(10.0))
        self.send_data = bytearray(
            [205, 235, 215, 44, 0x00, 0x00, 0x00, 0x00,
             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
             ])
        self.send_data2 = bytearray(
            [205, 235, 215, 32, 0x00, 0x00, 0x00, 0x00,
             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
             0x00, 0x00, 0x00, 0x00,
             ])
        self.monitor_server = monitor_server
        self.robot_pose_stamped = None
        self.galileo_status = galileo_status
        self.galileo_status_lock = galileo_status_lock

        self.env_sensor_data = EnvSensors()

        def update_envSensor(sensorData):
            self.env_sensor_data.header = sensorData.header
            self.env_sensor_data.temperature = sensorData.temperature #Centigrade
            self.env_sensor_data.rh = sensorData.rh #relative humidity %RH
            self.env_sensor_data.smoke = sensorData.smoke #ppm
            self.env_sensor_data.pm1_0 = sensorData.pm1_0 #ug/m^3
            self.env_sensor_data.pm2_5 = sensorData.pm2_5 #ug/m^3
            self.env_sensor_data.pm10 = sensorData.pm10 #ug/m^3
            self.env_sensor_data.lel = sensorData.lel #ppm
            self.env_sensor_data.noise = sensorData.noise #db

        if useEnvSensors == 0:
            self.envSensor_flag = False
        else:
            self.envSensor_sub = rospy.Subscriber("/bw_env_sensors/EnvSensorData", EnvSensors, update_envSensor)
            self.envSensor_flag = True

        self.odom_flag = False
        def update_odom(odom):
            self.odom_flag = True
            self.robot_pose_stamped = PoseStamped()
            self.robot_pose_stamped.pose = odom.pose.pose  # 更新坐标
            self.robot_pose_stamped.header = odom.header  # 更新坐标

        self.odom_sub = rospy.Subscriber("/bWmono/Odom", Odometry, update_odom)

    def stop(self):
        self.odom_sub.unregister()
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()

    def run(self):
        rate = rospy.Rate(30)
        self._stop.clear()
        num_i = 0
        while not self.stopped() and not rospy.is_shutdown():
            with self.galileo_status_lock:
                self.send_data[4:8] = map(ord, struct.pack('f', self.galileo_status.currentPosX))
                self.send_data[8:12] = map(ord, struct.pack('f', self.galileo_status.currentPosY))
                self.send_data[12:16] = map(ord, struct.pack('f', 0))
                self.send_data[16:20] = map(
                    ord, struct.pack('f', self.galileo_status.power))
                self.send_data[24:28] = map(
                    ord, struct.pack('f', self.galileo_status.currentAngle))
                self.send_data[28:32] = map(ord, struct.pack(
                    'i', self.galileo_status.targetStatus))
                self.send_data[32:36] = map(ord, struct.pack(
                    'i', self.galileo_status.targetNumID))
                self.send_data[40:44] = map(ord, struct.pack(
                    'i', self.galileo_status.loopStatus))
                self.send_data[36:40] = map(ord, struct.pack(
                    'i', self.galileo_status.chargeStatus))
                self.send_data[44:48] = map(
                    ord, struct.pack('f', self.galileo_status.targetDistance))

                if self.galileo_status.navStatus == 1:
                    statu0 = 0x01  # 导航状态
                else:
                    statu0 = 0x00
                if self.galileo_status.mapStatus:
                    statu1 = 0x02  # 建图状态
                else:
                    statu1 = 0x00
                if self.galileo_status.visualStatus != -1:
                    statu2 = 0x04  # 视觉系统状态
                else:
                    statu2 = 0x00
                if self.galileo_status.visualStatus == 1:
                    statu3 = 0x08  # 视觉系统状态
                else:
                    statu3 = 0x00
                if self.galileo_status.gcStatus == 1:
                    status4 = 0x10  # ORB_SLAM 内存回收状态
                else:
                    status4 = 0
                if self.galileo_status.gbaStatus == 1:
                    status5 = 0x20  # ORB_SLAM2 GBA状态，一般对应LoopClosing
                else:
                    status5 = 0x00
            self.send_data[20] = statu0 + statu1 + \
                statu2 + statu3 + status4 + status5
            self.send_data[3] = len(self.send_data) - 4
            self.monitor_server.sendto(bytes(self.send_data))

            if self.envSensor_flag and  num_i >= 3:
                num_i = 0
                self.send_data2[4:8] = map(ord, struct.pack('f', self.env_sensor_data.temperature))
                self.send_data2[8:12] = map(ord, struct.pack('f', self.env_sensor_data.rh))
                self.send_data2[12:16] = map(ord, struct.pack('f', self.env_sensor_data.smoke))
                self.send_data2[16:20] = map(ord, struct.pack('f', self.env_sensor_data.pm1_0))
                self.send_data2[20:24] = map(ord, struct.pack('f', self.env_sensor_data.pm2_5))
                self.send_data2[24:28] = map(ord, struct.pack('f', self.env_sensor_data.pm10))
                self.send_data2[28:32] = map(ord, struct.pack('f', self.env_sensor_data.lel))
                self.send_data2[32:36] = map(ord, struct.pack('f', self.env_sensor_data.noise))
                self.send_data2[3] = len(self.send_data2) - 4
                self.monitor_server.sendto(bytes(self.send_data2))
                # l = [hex(int(j)) for j in self.send_data2]
                # print(" ".join(l))
            num_i = num_i +1
            rate.sleep()
