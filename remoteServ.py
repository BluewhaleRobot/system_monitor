#!/usr/bin/env python
#coding:utf-8

import rospy
from std_msgs.msg import String, UInt32, Float64, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D, Pose
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

HOST = ''#should not be 127.0.0.1 or localhost
UserSocket_port=20001  #局域网udp命令监听端口
UserSocket_remote=None
UserSerSocket=None;
BUFSIZE = 1024

PACKAGE_HEADER = [205, 235, 215]
dataCache = []

cmd_pub=None
mapSave_pub=None

maxVel=1.4
maxTheta=7.2
mStatus = Status()
mStatus.brightness = 0.0
mStatus.imageStatus = False
mStatus.odomStatus = False
mStatus.orbStartStatus = False
mStatus.orbInitStatus = False
mStatus.power = 0.0
mStatus.orbScaleStatus = False
powerLow = 10.0

mStatusLock = threading.Lock()

dataCache = []
currentPose=Pose();
sendData=bytearray([205,235,215,20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00])
speed_cmd=Twist()

mapthread=None
navthread=None
control_flag=False

scaleOrbThread=None

def getDataFromReq(req):
    pass

def unpackReq(req):
    global dataCache
    res = []
    packageList = splitReq(req)
    # process the first package
    completeData = dataCache + packageList[0]
    packageList.remove(packageList[0])
    packageList =  splitReq(completeData) + packageList

    for count in range(0, len(packageList)):
        if len(packageList[count]) != 0 and len(packageList[count]) == packageList[count][0] + 1:
            res.append(packageList[count][1:])
    lastOne = packageList[-1:][0] # the last one
    if len(lastOne) == 0 or len(lastOne) != lastOne[0] + 1:
        dataCache = lastOne
    return res


def findPackageHeader(req):
    if len(req) < 3:
        return -1
    for count in range(0, len(req) - 2):
        if req[count] == PACKAGE_HEADER[0] and req[count + 1] == PACKAGE_HEADER[1] and req[count + 2] == PACKAGE_HEADER[2]:
            return count
    return -1

def splitReq(req):
    res = []
    startIndex = 0
    newIndex = 0
    while True:
        newIndex = findPackageHeader(req[startIndex:])
        if newIndex == -1:
            break
        res.append(req[startIndex: startIndex + newIndex])
        startIndex = newIndex + 3 + startIndex
    res.append(req[startIndex:])
    return res

def parseData(cmds):
    global cmd_pub, maxVel, maxTheta,mapthread,speed_cmd,control_flag
    global scaleOrbThread
    res = None
    for count in range(0, len(cmds)):
        if len(cmds[count])>0:
            control_flag=True
        #判断是否为关机命令
        if len(cmds[count])==2:
            if cmds[count][0]==0xaa and cmds[count][1]==0x44:
                print "system poweroff"
                status, output = commands.getstatusoutput('sudo shutdown -h now')

            if cmds[count][0]==ord('f'):
                # print "forward"
                speed_cmd.linear.x=maxVel*cmds[count][1]/100.0
                cmd_pub.publish(speed_cmd)
            elif cmds[count][0]==ord('b'):
                # print "back"
                speed_cmd.linear.x=-maxVel*cmds[count][1]/100.0
                cmd_pub.publish(speed_cmd)
            elif cmds[count][0]==ord('c'):
                # print "circleleft"
                speed_cmd.angular.z=maxTheta*cmds[count][1]/100.0/2.8
                cmd_pub.publish(speed_cmd)
            elif cmds[count][0]==ord('d'):
                # print "circleright"
                speed_cmd.angular.z=-maxTheta*cmds[count][1]/100.0/2.8
                cmd_pub.publish(speed_cmd)
            elif cmds[count][0]==ord('s'):
                # print "stop"
                                speed_cmd.linear.x = 0
                                speed_cmd.angular.z = 0
                                cmd_pub.publish(speed_cmd)
            elif cmds[count][0]==ord('V'):
                if cmds[count][1]==0:
                    # print "开启视觉"
                    if mapthread.stopped():
                        # print "开启视觉2"
                        mapthread.start()
                elif cmds[count][1]==1:
                    #  print "关闭视觉"
                     if not mapthread.stopped():
                        #  print "关闭视觉2"
                         mapthread.stop()
                elif cmds[count][1]==2:
                    #  print "保存地图"
                     mapSaveFlag=Bool()
                     mapSaveFlag.data=True
                     mapSave_pub.publish(mapSaveFlag)
                     if scaleOrbThread!=None:
                         scaleOrbThread.saveScale()
            elif cmds[count][0]==ord('m'):
                if cmds[count][1]==1:
                    # print "开始低速巡检"
                    if navthread.stopped():
                        navthread.setspeed(1)
                        navthread.start()
                if cmds[count][1]==2:
                    # print "开始中速巡检"
                    if navthread.stopped():
                        navthread.setspeed(2)
                        navthread.start()
                if cmds[count][1]==3:
                    # print "开始高速巡检"
                    if navthread.stopped():
                        navthread.setspeed(3)
                        navthread.start()
                if cmds[count][1]==4:
                    # print "关闭自主巡检"
                    if not navthread.stopped():
                        navthread.stop()
                    speed_cmd.linear.x = 0
                    speed_cmd.angular.z = 0
                    cmd_pub.publish(speed_cmd)
        #only for debug
        #print "recive orders"+str(cmds[count])
    return res

class UserSer(threading.Thread):
    #接收udp命令的socket
    def __init__(self):
        global HOST,CarSocket_port,UserSocket_port,CarSocket_remote,UserSocket_remote,CarSerSocket,UserSerSocket,BUFSIZE
        super(UserSer, self).__init__()
        self._stop = threading.Event()
        UserSerSocket=socket(AF_INET, SOCK_DGRAM)
        UserSerSocket.bind((HOST,UserSocket_port))
    def stop(self):
        global HOST,CarSocket_port,UserSocket_port,CarSocket_remote,UserSocket_remote,CarSerSocket,UserSerSocket,BUFSIZE
        if UserSerSocket!=None:
            UserSerSocket.close()
        self._stop.set()

    def stopped(self):
        global HOST,CarSocket_port,UserSocket_port,CarSocket_remote,UserSocket_remote,CarSerSocket,UserSerSocket,BUFSIZE
        return self._stop.isSet()

    def run(self):
        global HOST,CarSocket_port,UserSocket_port,CarSocket_remote,UserSocket_remote,CarSerSocket,UserSerSocket,BUFSIZE
        UserSerSocket.settimeout(2) #设置udp 2秒超时 等待
        while not self.stopped() and not rospy.is_shutdown():
            try:
                data, UserSocket_remote = UserSerSocket.recvfrom(BUFSIZE)
            except timeout:
                # print "timeout"
                continue
            #print "UserSocket get data"
            if not data: break
            #rospy.loginfo(data)
            dataList = []
            for c in data:
                dataList.append(ord(c))
            parseData(unpackReq(dataList))  ##处理命令数据
        self.stop();

class MapSer(threading.Thread):
    #orb_slam建图线程
    def __init__(self):
        super(MapSer, self).__init__()
        self._stop = threading.Event()
        self._stop.set()
        self.P=None
        self.psProcess=None
    def stop(self):
        global scaleOrbThread
        if scaleOrbThread !=None:
            scaleOrbThread.stop()
            scaleOrbThread =None

        if self.P!=None :
            # print str(self.P.pid)
            self.psProcess=psutil.Process(pid=self.P.pid)
            for child in self.psProcess.get_children(recursive=True):
                # print str(child.pid)
                child.kill()
            self.psProcess.kill()
        self.P=None
        self._stop.set()
        self.__init__()

    def stopped(self):
        return self._stop.isSet()

    def run(self):
        global scaleOrbThread
        self._stop.clear()
        cmd="roslaunch orb_slam2 map.launch"
        new_env=os.environ.copy()
        new_env['ROS_PACKAGE_PATH']='/home/xiaoqiang/Documents/ros/src:/opt/ros/jade/share:/opt/ros/jade/stacks:/home/xiaoqiang/Documents/ros/src/ORB_SLAM2/Examples/ROS'
        while not self.stopped() and not rospy.is_shutdown():

            time.sleep(0.1)
            if self.P==None:
                self.P=subprocess.Popen(cmd,shell=True,env=new_env)
                self.psProcess=psutil.Process(pid=self.P.pid)
                # print str(self.P.pid)
            else:
                if self.psProcess.is_running():
                    mStatusLock.acquire()
                    mStatus.orbStartStatus = True
                    mStatusLock.release()
                    if scaleOrbThread ==None:
                        scaleOrbThread = scaleOrb()
                        scaleOrbThread.start()
                else:
                    if scaleOrbThread !=None:
                        scaleOrbThread.stop()
                        scaleOrbThread =None
                    break
            # status, output = commands.getstatusoutput('roslaunch orb_slam2 map.launch')
        self.stop();

class NavSer(threading.Thread):
    #orb_slam建图线程
    def __init__(self):
        super(NavSer, self).__init__()
        self._stop = threading.Event()
        self._stop.set()
        self.P=None
        self.psProcess=None
        self.speed=1;
    def stop(self):
        if self.P!=None :
            # print str(self.P.pid)
            self.psProcess=psutil.Process(pid=self.P.pid)
            for child in self.psProcess.get_children(recursive=True):
                # print str(child.pid)
                child.kill()
            self.psProcess.kill()
        self.P=None
        self._stop.set()
        self.__init__()

    def stopped(self):
        return self._stop.isSet()

    def setspeed(self,speed):
        self.speed=speed

    def run(self):
        self._stop.clear()
        if self.speed==1:
            cmd="roslaunch nav_test  tank_blank_map1.launch"
        elif self.speed==2:
            cmd="roslaunch nav_test  tank_blank_map2.launch"
        elif self.speed==3:
            cmd="roslaunch nav_test  tank_blank_map3.launch"

        new_env=os.environ.copy()
        new_env['ROS_PACKAGE_PATH']='/home/xiaoqiang/Documents/ros/src:/opt/ros/jade/share:/opt/ros/jade/stacks:/home/xiaoqiang/Documents/ros/src/ORB_SLAM2/Examples/ROS'
        while not self.stopped() and not rospy.is_shutdown():

            time.sleep(0.1)
            if self.P==None:
                self.P=subprocess.Popen(cmd,shell=True,env=new_env)
                self.psProcess=psutil.Process(pid=self.P.pid)
                # print str(self.P.pid)
            else:
                if self.psProcess.is_running():
                    mStatusLock.acquire()
                    mStatus.orbStartStatus = True
                    mStatusLock.release()
                else:
                    break
            # status, output = commands.getstatusoutput('roslaunch orb_slam2 map.launch')
        self.stop();


def getPower(power):
    mStatusLock.acquire()
    mStatus.power = power.data
    #if power.data < powerLow and not os.path.isfile(powerFlagFilePath) and power.data > 0.1:
    mStatusLock.release()

def getImage(image):
    mStatusLock.acquire()
    if image != None:
        mStatus.imageStatus = True
    else:
        mStatus.imageStatus = False
    mStatusLock.release()

def getOdom(odom):
    global currentPose
    mStatusLock.acquire()
    if odom != None:
        mStatus.odomStatus = True
        currentPose=odom.pose.pose; #更新坐标

    else:
        mStatus.odomStatus = False
    mStatusLock.release()

def getOrbStartStatus(orb_frame):
    mStatusLock.acquire()
    if orb_frame != None:
        mStatus.orbStartStatus = True
    else:
        mStatus.orbStartStatus = False
    mStatusLock.release()

def getOrbTrackingFlag(cam_pose):
    mStatusLock.acquire()
    if cam_pose != None:
        mStatus.orbInitStatus = True
    else:
        mStatus.orbInitStatus = False
    mStatusLock.release()

def broadcast():
    global reportPub, cmd_pub,mapSave_pub
    rospy.init_node("broadcast", anonymous=True)
    rospy.Subscriber("/xqserial_server/Power", Float64, getPower)
    rospy.Subscriber("/usb_cam/image_raw", Image, getImage)
    rospy.Subscriber("/odom_combined", Odometry, getOdom)
    rospy.Subscriber("/ORB_SLAM/Camera", Pose, getOrbTrackingFlag)
    rospy.Subscriber("/ORB_SLAM/Frame", Image, getOrbStartStatus)
    cmd_pub = rospy.Publisher('/cmd_vel', Twist , queue_size=0)
    mapSave_pub = rospy.Publisher('/map_save', Bool , queue_size=0)

class scaleOrb(threading.Thread):

    def __init__(self):
        super(scaleOrb, self).__init__()
        self._stop = threading.Event()
        self.car_odoms = 0.0
        self.cam_odoms = 0.0
        self.car_lastPose = None
        self.cam_lastPose = None
        self.scale=5.
        self.carPoseLock = threading.Lock()
        self.camPoseLock = threading.Lock()
        self.scaleLock = threading.Lock()
        self.carbegin_flag=True
        self.cambegin_flag=True
    def stop(self):
        global scaleOrbThread
        self._stop.set()


    def stopped(self):
        return self._stop.isSet()

    def run(self):
        def updateCar(pose2d):
            self.carPoseLock.acquire()
            self.scaleLock.acquire()
            if self.carbegin_flag:
                self.car_lastPose = pose2d
                self.carbegin_flag=False
            self.car_odoms+=math.sqrt((pose2d.x-self.car_lastPose.x)*(pose2d.x-self.car_lastPose.x)+(pose2d.y-self.car_lastPose.y)*(pose2d.y-self.car_lastPose.y))
            self.scaleLock.release()
            self.car_lastPose = pose2d
            self.carPoseLock.release()

        def updateCam(pose):
            self.camPoseLock.acquire()
            self.scaleLock.acquire()
            if self.cambegin_flag:
                self.cam_lastPose = pose
                self.cambegin_flag=False
            self.cam_odoms+=math.sqrt((pose.position.x-self.cam_lastPose.position.x)*(pose.position.x-self.cam_lastPose.position.x)+(pose.position.z-self.cam_lastPose.position.z)*(pose.position.z-self.cam_lastPose.position.z))
            self.scaleLock.release()
            self.cam_lastPose= pose
            self.camPoseLock.release()
        while not mStatus.orbInitStatus and not rospy.is_shutdown():
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
        if self.cam_odoms>0.01:
            self.scale=self.car_odoms/self.cam_odoms
        else:
            self.scale=5.0
        fp3=open("/home/xiaoqiang/slamdb/scale.txt",'a+')
        fp3.write(str(self.scale))#+" "+str(self.car_odoms)+" "+str(self.cam_odoms))
        fp3.write('\n')
        fp3.close
        self.scaleLock.release()



if __name__ == "__main__":
    broadcast()
    rate = rospy.Rate(10)
    #配置udp广播
    MYPORT = 22001  #广播端口
    s = socket(AF_INET, SOCK_DGRAM)
    s.bind(('', 0))
    s.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)
    #开启udp接收监听线程
    UserSerThread = UserSer()
    UserSerThread.start()
    mapthread = MapSer()
    navthread = NavSer()
    i=10
    ii=20
    while not rospy.is_shutdown():
        if not control_flag and ii>12 and  ii<17:
            speed_cmd.linear.x = 0
            speed_cmd.angular.z = 0
            cmd_pub.publish(speed_cmd)
        #每两秒心跳维护一次
        if ii==20:
            ii=0
            control_flag=False
        ii+=1
        #持续反馈状态
        if UserSocket_remote!=None and UserSerSocket!=None:

            sendData[4:8]=map(ord,struct.pack('f',currentPose.position.x))
            sendData[8:12]=map(ord,struct.pack('f',currentPose.position.y))
            sendData[12:16]=map(ord,struct.pack('f',currentPose.position.z))
            sendData[16:20]=map(ord,struct.pack('f',mStatus.power))
            if mStatus.odomStatus :
                statu0=0x01 #混合里程计
            else:
                statu0=0x00
            if mStatus.imageStatus :
                statu1=0x02 #视觉摄像头
            else:
                statu1=0x00
            if mStatus.orbStartStatus :
                statu2=0x04 #视觉系统状态
            else:
                statu2=0x00
            if mStatus.orbInitStatus :
                statu3=0x08 #视觉系统状态
            else:
                statu3=0x00
            sendData[20]=statu0+statu1+statu2+statu3

            UserSerSocket.sendto(bytes(sendData),UserSocket_remote)

        #每秒广播一次
        if i==10:
            i=0;
            data = "xq"
            #发送广播包
            try:
                s.sendto(data, ('<broadcast>', MYPORT))
            except:
                continue
            # clear data
            mStatus.power = 0.0
            mStatus.orbInitStatus = False
            mStatus.orbStartStatus = False
            mStatus.imageStatus = False
            mStatus.odomStatus = False
            # print "getyou4"
            # print str(sendData[20])
        i+=1;
        rate.sleep()
    if not mapthread.stopped():
        mapthread.stop();
    UserSerThread.stop()
