#!/usr/bin/env python
#coding:utf-8

import rospy
from std_msgs.msg import String, UInt32, Float64, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from system_monitor.msg import *
import threading
import os
import sys
from socket import *
import commands

HOST = ''#should not be 127.0.0.1 or localhost
UserSocket_port=20001  #局域网udp命令监听端口
UserSocket_remote=None
UserSerSocket=None;
BUFSIZE = 1024

PACKAGE_HEADER = [205, 235, 215]
dataCache = []


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
    res = None
    for count in range(0, len(cmds)):
        #判断是否为关机命令
        if cmds[count][0]==0xaa and cmds[count][1]==0x44:
            print "system poweroff"
            status, output = commands.getstatusoutput('sudo shutdown -h now')
        # cmds[count][0] 为目标编号,取值为0到127
        # cmds[count][1] 为速度 取值为0到100

        # todo 发布目标编号
        # todo 发布目标编号
        # todo 发布目标编号




        #only for debug
        print "TargetNum %d" %(cmds[count][0])
        print "MoveSpeed %d" %(cmds[count][1])
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
        while not self.stopped() and not rospy.is_shutdown():
            data, UserSocket_remote = UserSerSocket.recvfrom(BUFSIZE)
            #print "UserSocket get data"
            if not data: break
            #rospy.loginfo(data)
            dataList = []
            for c in data:
                dataList.append(ord(c))
            parseData(unpackReq(dataList))  ##处理命令数据
        self.stop();

def getPower(power):
    mStatusLock.acquire()
    mStatus.power = power.data
    #if power.data < powerLow and not os.path.isfile(powerFlagFilePath) and power.data > 0.1:
    mStatusLock.release()

def getOrbTrackingFlag(cam_pose):
    mStatusLock.acquire()
    if cam_pose != None:
        mStatus.orbInitStatus = True
    else:
        mStatus.orbInitStatus = False
    mStatusLock.release()

def broadcast():
    global reportPub
    rospy.init_node("broadcast", anonymous=True)
    rospy.Subscriber("/xqserial_server/Power", Float64, getPower)
    rospy.Subscriber("/ORB_SLAM/Camera", Pose, getOrbTrackingFlag)


if __name__ == "__main__":
    broadcast()
    rate = rospy.Rate(1)
    #配置udp广播
    MYPORT = 22001  #广播端口
    s = socket(AF_INET, SOCK_DGRAM)
    s.bind(('', 0))
    s.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)
    #开启udp接收监听线程
    UserSerThread = UserSer()
    UserSerThread.start()
    while not rospy.is_shutdown():

        #每秒广播一次电压和ORB_SLAM tracker 状态
        if(mStatus.orbInitStatus):
            data = "xq%5.2f1" % mStatus.power
        else:
            data = "xq%5.2f0" % mStatus.power
        #发送广播包
        s.sendto(data, ('<broadcast>', MYPORT))

        #持续反馈状态
        if UserSocket_remote!=None and UserSerSocket!=None:
            UserSerSocket.sendto(data,UserSocket_remote)

        # clear data
        mStatus.power = 0.0
        mStatus.orbInitStatus = False
        rate.sleep()
    UserSerThread.stop()
