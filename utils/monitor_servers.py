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
from galileo_serial_server.msg import GalileoNativeCmds, GalileoStatus
from req_parser import ReqParser
from config import MAX_VEL, MAX_THETA
from scale_orb import ScaleORB
from nav_task import NavTask


class UserSer(threading.Thread):
    # 接收udp命令的socket
    def __init__(self, pubs, ROBOT_STATUS_LOCK, ROBOT_STATUS, HOST='', USERSOCKET_PORT=20001, BUFSIZE=1024):
        super(UserSer, self).__init__()
        self.HOST = HOST
        self.USERSOCKET_PORT = USERSOCKET_PORT
        self.BUFSIZE = BUFSIZE
        self._stop = threading.Event()
        self.USERSER_SOCKET = socket(AF_INET, SOCK_DGRAM)
        self.USERSER_SOCKET.bind((self.HOST, self.USERSOCKET_PORT))
        self.parser = ReqParser()
        self.SPEED_CMD = Twist()

        self.GLOBAL_MOVE_PUB = pubs["GLOBAL_MOVE_PUB"]
        self.CMD_VEL_PUB = pubs["CMD_VEL_PUB"]
        self.MAPSAVE_PUB = pubs["MAPSAVE_PUB"]
        self.ELEVATOR_PUB = pubs["ELEVATOR_PUB"]
        self.TILT_PUB = pubs["TILT_PUB"]

        self.MAP_THREAD = MapSer(ROBOT_STATUS_LOCK, ROBOT_STATUS)
        self.NAV_THREAD = NavSer(ROBOT_STATUS_LOCK, ROBOT_STATUS)
        self.NAV_LASTTIME = rospy.Time(0)
        self.NAV_FLAG = False
        self.USERSOCKET_REMOTE = None

        self.nav_task = None
        self.log("service started")

        def get_galileo_cmds(cmds):
            self.parse_data([map(lambda x: ord(x), list(cmds.data))])

        rospy.Subscriber('/galileo/cmds', GalileoNativeCmds, get_galileo_cmds)

    def log(self, info):
        with open("log.txt", "a+") as log_file:
            log_file.write(info + '\n')

    def stop(self):
        if self.USERSER_SOCKET != None:
            self.USERSER_SOCKET.close()
        if not self.MAP_THREAD.stopped():
            self.MAP_THREAD.stop()
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()

    def run(self):
        self.USERSER_SOCKET.settimeout(2)  # 设置udp 2秒超时 等待
        while not self.stopped() and not rospy.is_shutdown():
            try:
                data, self.USERSOCKET_REMOTE = self.USERSER_SOCKET.recvfrom(
                    self.BUFSIZE)
            except timeout:
                continue
            if not data:
                break
            dataList = []
            for c in data:
                dataList.append(ord(c))
            self.parse_data(self.parser.unpack_req(dataList))  # 处理命令数据
        self.stop()

    def parse_data(self, cmds):
        res = None
        time_now = rospy.Time.now()
        for count in range(0, len(cmds)):
            if len(cmds[count]) > 0:
                self.CONTROL_FLAG = True
                global_move_flag = Bool()
                global_move_flag.data = True
                self.GLOBAL_MOVE_PUB.publish(global_move_flag)
            # 判断是否为关机命令
            if len(cmds[count]) == 2:
                global_move_flag = Bool()
                global_move_flag.data = True

                temp_scale = 1.0
                if abs(self.SPEED_CMD.linear.x) < 1.0:
                    temp_scale = 1.0
                else:
                    temp_scale = abs(self.SPEED_CMD.linear.x)

                if cmds[count][0] == 0xaa and cmds[count][1] == 0x44:
                    print "system poweroff"
                    status, output = commands.getstatusoutput(
                        'sudo shutdown -h now')

                if cmds[count][0] == ord('f'):
                    print "forward"
                    self.GLOBAL_MOVE_PUB.publish(global_move_flag)
                    self.SPEED_CMD.linear.x = MAX_VEL * cmds[count][1] / 100.0
                    self.CMD_VEL_PUB.publish(self.SPEED_CMD)
                elif cmds[count][0] == ord('b'):
                    print "back"
                    self.GLOBAL_MOVE_PUB.publish(global_move_flag)
                    self.SPEED_CMD.linear.x = -MAX_VEL * cmds[count][1] / 100.0
                    self.CMD_VEL_PUB.publish(self.SPEED_CMD)
                elif cmds[count][0] == ord('c'):
                    print "circleleft"
                    self.GLOBAL_MOVE_PUB.publish(global_move_flag)
                    if cmds[count][1] > 1:
                        self.SPEED_CMD.angular.z = max(
                            0.4, MAX_THETA * cmds[count][1] / 100.0 / temp_scale)
                    else:
                        self.SPEED_CMD.angular.z = MAX_THETA * \
                            cmds[count][1] / 100.0 / temp_scale
                    #self.SPEED_CMD.angular.z = MAX_THETA * cmds[count][1]/100.0/2.8
                    self.CMD_VEL_PUB.publish(self.SPEED_CMD)
                elif cmds[count][0] == ord('d'):
                    print "circleright"
                    self.GLOBAL_MOVE_PUB.publish(global_move_flag)
                    if cmds[count][1] > 1:
                        self.SPEED_CMD.angular.z = min(
                            -0.4, -MAX_THETA * cmds[count][1] / 100.0 / temp_scale)
                    else:
                        self.SPEED_CMD.angular.z = -MAX_THETA * \
                            cmds[count][1] / 100.0 / temp_scale
                    #self.SPEED_CMD.angular.z = -MAX_THETA * cmds[count][1]/100.0/2.8
                    self.CMD_VEL_PUB.publish(self.SPEED_CMD)
                elif cmds[count][0] == ord('s'):
                    print "stop"
                    self.SPEED_CMD.linear.x = 0
                    self.SPEED_CMD.angular.z = 0
                    self.CMD_VEL_PUB.publish(self.SPEED_CMD)
                elif cmds[count][0] == ord('V'):
                    if cmds[count][1] == 0:
                        print "开启视觉"
                        if self.MAP_THREAD.stopped():
                            print "开启视觉2"
                            self.MAP_THREAD.update = False
                            self.MAP_THREAD.start()
                    elif cmds[count][1] == 1:
                        print "关闭视觉"
                        if not self.MAP_THREAD.stopped():
                            print "关闭视觉2"
                            self.MAP_THREAD.stop()
                        os.system("pkill -f odom2map.py")
                        os.system("pkill -f navGuide.py")
                        os.system("pkill -f ORB_SALM")
                        os.system("pkill -f map_server")
                        os.system("pkill -f move_base")
                        os.system("pkill -f odom_map_broadcaster")
                    elif cmds[count][1] == 2:
                        print "保存地图"
                        mapSaveFlag = Bool()
                        mapSaveFlag.data = True
                        self.MAPSAVE_PUB.publish(mapSaveFlag)
                        if self.MAP_THREAD.SCALE_ORB_THREAD != None:
                            self.MAP_THREAD.SCALE_ORB_THREAD.saveScale()
                    elif cmds[count][1] == 3:
                        print("更新地图")
                        if self.MAP_THREAD.stopped():
                            print "开启视觉2"
                            self.MAP_THREAD.update = True
                            self.MAP_THREAD.start()
                elif cmds[count][0] == ord('h'):
                    elePose = UInt32()
                    elePose.data = cmds[count][1]
                    self.ELEVATOR_PUB.publish(elePose)
                elif cmds[count][0] == ord('m'):
                    time1_diff = time_now - self.NAV_LASTTIME
                    if cmds[count][1] == 1:
                        if time1_diff.to_sec() < 30:
                            continue
                        print "开始低速巡检"
                        self.NAV_LASTTIME = time1_diff
                        tilt_degree = Int16()
                        tilt_degree.data = -19
                        self.TILT_PUB.publish(tilt_degree)
                        if self.NAV_THREAD.stopped():
                            self.NAV_THREAD.setspeed(1)
                            self.NAV_THREAD.start()
                            self.nav_task = NavTask()

                    if cmds[count][1] == 2:
                        if time1_diff.to_sec() < 30:
                            continue
                        print "开始中速巡检"
                        self.NAV_LASTTIME = time1_diff
                        tilt_degree = Int16()
                        tilt_degree.data = -19
                        self.TILT_PUB.publish(tilt_degree)
                        if self.NAV_THREAD.stopped():
                            self.NAV_THREAD.setspeed(2)
                            self.NAV_THREAD.start()
                            self.nav_task = NavTask()
                    if cmds[count][1] == 3:
                        if time1_diff.to_sec() < 30:
                            continue
                        print "开始高速巡检"
                        self.NAV_LASTTIME = time1_diff
                        tilt_degree = Int16()
                        tilt_degree.data = -19
                        self.TILT_PUB.publish(tilt_degree)
                        if self.NAV_THREAD.stopped():
                            self.NAV_THREAD.setspeed(3)
                            self.NAV_THREAD.start()
                            self.nav_task = NavTask()
                    if cmds[count][1] == 0:
                        if time1_diff.to_sec() < 30:
                            continue
                        print "开启视觉，不巡检"
                        self.NAV_LASTTIME = time1_diff
                        tilt_degree = Int16()
                        tilt_degree.data = -19
                        self.TILT_PUB.publish(tilt_degree)
                        if self.NAV_THREAD.stopped():
                            self.NAV_THREAD.setspeed(0)
                            self.NAV_THREAD.start()
                            self.nav_task = NavTask()
                    if cmds[count][1] == 4:
                        print "关闭自主巡检"
                        if self.nav_task is not None:
                            self.nav_task.shutdown()
                            self.nav_task = None
                        tilt_degree = Int16()
                        tilt_degree.data = 0
                        self.TILT_PUB.publish(tilt_degree)
                        if not self.NAV_THREAD.stopped():
                            self.NAV_THREAD.stop()
                        self.SPEED_CMD.linear.x = 0
                        self.SPEED_CMD.angular.z = 0
                        self.CMD_VEL_PUB.publish(self.SPEED_CMD)
                        self.NAV_FLAG = False
                        os.system("pkill -f odom2map.py")
                        os.system("pkill -f navGuide.py")
                        os.system("pkill -f ORB_SLAM")
                        os.system("pkill -f map_server")
                        os.system("pkill -f move_base")
                        os.system("pkill -f odom_map_broadcaster")
                elif cmds[count][0] == ord('g'):
                    self.nav_task.set_goal(cmds[count][1])
                elif cmds[count][0] == ord('i'):
                    if cmds[count][1] == 0:
                        self.nav_task.pause()
                    if cmds[count][1] == 1:
                        self.nav_task.resume()
                    if cmds[count][1] == 2:
                        self.nav_task.cancel_goal()
        return res

    def sendto(self, data):
        try:
            self.USERSER_SOCKET.sendto(bytes(data), self.USERSOCKET_REMOTE)
        except:
            print "remote disconnect !\n"

    def get_connection_status(self):
        if self.USERSER_SOCKET != None and self.USERSOCKET_REMOTE != None:
            return True
        else:
            return False


class MapSer(threading.Thread):
    # orb_slam建图线程
    def __init__(self, ROBOT_STATUS_LOCK, ROBOT_STATUS, update=False):
        super(MapSer, self).__init__()
        self._stop = threading.Event()
        self._stop.set()
        self.P = None
        self.psProcess = None
        self.SCALE_ORB_THREAD = None
        self.ROBOT_STATUS_LOCK = ROBOT_STATUS_LOCK
        self.ROBOT_STATUS = ROBOT_STATUS
        self.update = update

    def stop(self):
        if self.SCALE_ORB_THREAD != None:
            self.SCALE_ORB_THREAD.stop()
            self.SCALE_ORB_THREAD = None

        if self.P != None:
            self.psProcess = psutil.Process(pid=self.P.pid)
            for child in self.psProcess.children(recursive=True):
                child.kill()
            self.psProcess.kill()
        self.P = None
        self._stop.set()
        self.__init__(self.ROBOT_STATUS_LOCK, self.ROBOT_STATUS)

    def stopped(self):
        return self._stop.isSet()

    def run(self):
        self._stop.clear()
        cmd = "roslaunch ORB_SLAM2 map.launch"
        if self.update:
            cmd = "roslaunch ORB_SLAM2 map.launch"
        new_env = os.environ.copy()
        new_env['ROS_PACKAGE_PATH'] = '/home/xiaoqiang/Documents/ros/src:/opt/ros/kinetic/share:/opt/ros/kinetic/stacks:/home/xiaoqiang/Documents/ros/src/ORB_SLAM2/Examples/ROS'
        while not self.stopped() and not rospy.is_shutdown():
            if self.P == None and not self.stopped():
                self.P = subprocess.Popen(cmd, shell=True, env=new_env)
                self.psProcess = psutil.Process(pid=self.P.pid)
                # print str(self.P.pid)
            else:
                if self.psProcess.is_running():
                    self.ROBOT_STATUS_LOCK.acquire()
                    self.ROBOT_STATUS.orbStartStatus = True
                    self.ROBOT_STATUS_LOCK.release()
                    if self.SCALE_ORB_THREAD == None:
                        self.SCALE_ORB_THREAD = ScaleORB(self.ROBOT_STATUS)
                        self.SCALE_ORB_THREAD.start()
                else:
                    if self.SCALE_ORB_THREAD != None:
                        self.SCALE_ORB_THREAD.stop()
                        self.SCALE_ORB_THREAD = None
                    break
            time.sleep(0.1)
        self.stop()


class NavSer(threading.Thread):
    # orb_slam建图线程
    def __init__(self, ROBOT_STATUS_LOCK, ROBOT_STATUS):
        super(NavSer, self).__init__()
        self._stop = threading.Event()
        self._stop.set()
        self.P = None
        self.psProcess = None
        self.speed = 1
        self.ROBOT_STATUS_LOCK = ROBOT_STATUS_LOCK
        self.ROBOT_STATUS = ROBOT_STATUS

    def stop(self):
        if self.P != None:
            # print str(self.P.pid)
            self.psProcess = psutil.Process(pid=self.P.pid)
            for child in self.psProcess.children(recursive=True):
                # print str(child.pid)
                child.kill()
            self.psProcess.kill()
        self.P = None
        self._stop.set()
        self.__init__(self.ROBOT_STATUS_LOCK, self.ROBOT_STATUS)

    def stopped(self):
        return self._stop.isSet()

    def setspeed(self, speed):
        self.speed = speed

    def run(self):
        self._stop.clear()
        if self.speed == 1:
            cmd = "roslaunch nav_test tank_blank_map1.launch"
        elif self.speed == 2:
            cmd = "roslaunch nav_test tank_blank_map2.launch"
        elif self.speed == 3:
            cmd = "roslaunch nav_test tank_blank_map3.launch"
        elif self.speed == 0:
            cmd = "roslaunch nav_test tank_blank_map0.launch"

        new_env = os.environ.copy()
        new_env['ROS_PACKAGE_PATH'] = '/home/xiaoqiang/Documents/ros/src:/opt/ros/kinetic/share:/opt/ros/kinetic/stacks:/home/xiaoqiang/Documents/ros/src/ORB_SLAM2/Examples/ROS'
        while not self.stopped() and not rospy.is_shutdown():
            if self.P == None and not self.stopped():
                self.P = subprocess.Popen(cmd, shell=True, env=new_env)
                self.psProcess = psutil.Process(pid=self.P.pid)
            else:
                if self.psProcess.is_running():
                    self.ROBOT_STATUS_LOCK.acquire()
                    self.ROBOT_STATUS.orbStartStatus = True
                    self.ROBOT_STATUS_LOCK.release()
                else:
                    break
            time.sleep(0.1)
        self.stop()
