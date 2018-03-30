#!/usr/bin/env python
#coding:utf-8
'''
已知R 已知T 已知scale 输出融合里程计
'''

import rospy
from std_msgs.msg import String, UInt32, Float64, Bool,Int32, Int16
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
import numpy as  np
import scipy.linalg

getTrackThread = None
orbInitFlag = False
orbStartFlag =False
mStatusLock = threading.Lock()

velPub = None
ClearPub = None
transform = None
lisener=None
odom2map_tf=None
odom2map_tf2=None
scale = 0.
prviousCamPose = None
prviousstamp = None
tf_rot=np.array([[ 0., 0.03818382, 0.99927073],[ -1., 0.,0.], [0., -0.99927073, 0.03818382]])
tf_trans=np.array([0.0,0.0,0.])
Mhf=np.identity(4)


begin_flag=True
globalMovePub=None

camUpdateFlag=False
carUpdateFlag=False

cam_currentPose=Pose()
cam_currentTime=None
car_currentPose=Pose()
car_currentTime=None
cam_lastPose=Pose()

barFlag=False
navFlag_pub=None

enableMoveFlag=True
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
    global orbInitFlag,mStatusLock,orbStartFlag
    mStatusLock.acquire()
    orbInitFlag=system_status.orbInitStatus
    orbStartFlag=system_status.orbStartStatus
    mStatusLock.release()

def carOdom(odom):
    global car_currentPose,car_currentTime,mStatusLock,begin_flag,carUpdateFlag
    mStatusLock.acquire()
    carUpdateFlag=True
    car_currentPose=odom.pose.pose
    car_currentTime=rospy.Time.now()
    if begin_flag:
        car_lastPose=car_currentPose
    mStatusLock.release()


def cameraOdom(campose):
    global cam_currentPose,cam_currentTime,mStatusLock,begin_flag,camUpdateFlag
    mStatusLock.acquire()
    camUpdateFlag=True
    cam_currentPose=campose
    cam_currentTime=rospy.Time.now()
    if begin_flag:
        cam_lastPose=cam_currentPose
    mStatusLock.release()


def doTfPub():
    global camUpdateFlag,carUpdateFlag,mStatusLock
    global cam_currentPose,cam_currentTime
    global car_currentPose,car_currentTime
    global car_lastPose,cam_lastPose
    global begin_flag,scale,globalMovePub,velPub
    global Mhf,tf_rot,tf_trans,scale
    global odom2map_tf,odom2map_tf2
    global navFlag_pub,ClearPub,enableMoveFlag

    time_now=rospy.Time.now()

    Odom = Odometry()

    Odom.header.stamp = time_now

    Odom.header.frame_id = "odom_combined"

    mStatusLock.acquire()

    camflag=camUpdateFlag
    carflag=carUpdateFlag

    if begin_flag:
        cam_lastPose=cam_currentPose
        car_lastPose=car_currentPose

    time0_diff=cam_currentTime-car_currentTime

    if camUpdateFlag and abs(time0_diff.to_sec())<0.05 :

        Tad=np.array([cam_currentPose.position.x,cam_currentPose.position.y,cam_currentPose.position.z])
        q=[cam_currentPose.orientation.x,cam_currentPose.orientation.y,cam_currentPose.orientation.z,cam_currentPose.orientation.w]
        M=tf.transformations.quaternion_matrix(q)
        Rad=M[:3,:3]
        #为了简化计算，下文的计算中base_link 和base_footprint被看成是相同的坐标系
        #转到odom_combined，得到camera在odom_combined中的pose
        Rbd=tf_rot.dot(Rad)
        Tbd=scale*(tf_rot.dot(Tad))+tf_trans
        Tbd[2]=0;
        #由camera 的 pose 得到 base_footprint 的pose，这也是下文要发布的pose
        Rdc=tf_rot.T
        Tdc=-1/scale*(Rdc.dot(tf_trans))
        Rbc=Rbd.dot(Rdc)
        Tbc=scale*(Rbd.dot(Tdc))+Tbd
        #Tbc[2]=0.0

        Mho=np.identity(4)
        Mho[:3,:3]=Rbc
        Mho[:3,3]=Tbc

        Mof=np.identity(4)
        q=[car_currentPose.orientation.x,car_currentPose.orientation.y,car_currentPose.orientation.z,car_currentPose.orientation.w]
        M=tf.transformations.quaternion_matrix(q)
        Rfo=M[:3,:3]
        Tfo=np.array([car_currentPose.position.x,car_currentPose.position.y,car_currentPose.position.z])
        Mof[:3,:3]=Rfo.T
        Mof[:3,3]=-Rfo.T.dot(Tfo)

        Mhf=Mho.dot(Mof)

        if begin_flag:
            begin_flag=False

    time1_diff=time_now-car_currentTime
    time2_diff=time_now-cam_currentTime
    car_diff=abs(car_currentPose.position.x-car_lastPose.position.x)+abs(car_currentPose.position.y-car_lastPose.position.y)
    if car_diff>4.0 or time2_diff.to_sec()>180. or time1_diff.to_sec()>5.:
        if car_diff>4.0 or enableMoveFlag:
            if barFlag and not begin_flag:
                if odom2map_tf != None:
                    T=Mhf[:3,3]
                    M=np.identity(4)
                    M[:3,:3]=Mhf[:3,:3]
                    q=tf.transformations.quaternion_from_matrix(M)
                    odom2map_tf.sendTransform(T,q,
                        time_now,
                        "/odom",
                        "/map")
                if odom2map_tf2 != None:
                    M=np.identity(4)
                    M[:3,:3]=tf_rot
                    q=tf.transformations.quaternion_from_matrix(M)
                    odom2map_tf2.sendTransform(tf_trans,q,
                        time_now,
                        "/ORB_SLAM/World",
                        "/map")
                mStatusLock.release()
                return
            #发布navFlag
            if navFlag_pub!=None:
                navFlag=Bool()
                navFlag.data=True
                navFlag_pub.publish(navFlag)

            #发布全局禁止ｆｌａｇ
            globalMoveFlag=Bool()
            globalMoveFlag.data=False
            globalMovePub.publish(globalMoveFlag)
            CarTwist = Twist()
            velPub.publish(CarTwist)
            car_lastPose=car_currentPose
            cam_currentTime=rospy.Time.now()
            car_currentTime=rospy.Time.now()
            mStatusLock.release()
            return

    if carUpdateFlag:
        car_lastPose=car_currentPose

    camUpdateFlag=False
    carUpdateFlag=False

    mStatusLock.release()

    if begin_flag:
        return

    if odom2map_tf != None:
        T=Mhf[:3,3]
        M=np.identity(4)
        M[:3,:3]=Mhf[:3,:3]
        q=tf.transformations.quaternion_from_matrix(M)
        odom2map_tf.sendTransform(T,q,
            time_now,
            "/odom",
            "/map")
    if odom2map_tf2 != None:
        M=np.identity(4)
        M[:3,:3]=tf_rot
        q=tf.transformations.quaternion_from_matrix(M)
        odom2map_tf2.sendTransform(tf_trans,q,
            time_now,
            "/ORB_SLAM/World",
            "/map")


def dealCarStatus(carStatu):
    global barFlag
    status=carStatu.data
    if status==2:
        barFlag=True
    else:
        barFlag=False

def init():
    global orbInitFlag,scale,odom2map_tf,odom2map_tf2,orbStartFlag
    global velPub,getTrackThread,globalMovePub
    global cam_currentTime,car_currentTime
    global barFlag,navFlag_pub,ClearPub,enableMoveFlag
    barFlag=False

    rospy.init_node("odom2map", anonymous=True)
    cam_currentTime=rospy.Time.now()
    car_currentTime=rospy.Time.now()
    scaleOrbFlag=rospy.get_param('/orb2base_scaleFlag',True)
    enableMoveFlag = rospy.get_param("~enableMoveFlag", True)
    if scaleOrbFlag:
        #load scale value from scale.txt
        #file_path=os.path.split(os.path.realpath(__file__))[0]
        #fp3=open("%s/scale.txt"%(file_path),'r+')
        fp3=open("/home/xiaoqiang/slamdb/scale.txt",'r+')
        for line in fp3:
            value_list=line.split(" ")
        scale=float(value_list[0])
        if scale<=0.000001:
            scale=1.05
        rospy.set_param('/orb2base_scale',scale)
        fp3.close
        print "scale: "+str(scale)
    odom2map_tf=tf.TransformBroadcaster()
    odom2map_tf2=tf.TransformBroadcaster()
    rospy.Subscriber("/system_monitor/report", Status, systemStatusHandler)
    rospy.Subscriber("/ORB_SLAM/Camera", Pose, cameraOdom)
    rospy.Subscriber("/xqserial_server/Odom", Odometry, carOdom)
    rospy.Subscriber("/xqserial_server/StatusFlag", Int32, dealCarStatus)
    globalMovePub = rospy.Publisher('/globalMoveFlag', Bool , queue_size=1)
    navFlag_pub = rospy.Publisher('/nav_setStop', Bool , queue_size=0)
    velPub = rospy.Publisher('/cmd_vel', Twist , queue_size=5)

    ClearPub = rospy.Publisher("/kinect/clearall", Bool, queue_size=5)
    #确保orb 已经启动
    while not orbStartFlag and not rospy.is_shutdown():
        print "oups! 1"
        time.sleep(1)
    #确保orb　已经 track
    time.sleep(4)
    print "oups! 2"
    globalMoveFlag=Bool()
    globalMoveFlag.data=True
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
    rate = rospy.Rate(40)
    tilt_pub = rospy.Publisher('/set_tilt_degree', Int16 , queue_size=1)
    i=0
    while not rospy.is_shutdown():
        doTfPub()
        if i>4:
            i=0
            tilt_degree=Int16()
            tilt_degree.data=-19
            tilt_pub.publish(tilt_degree)
        rate.sleep()
    if getTrackThread != None:
        getTrackThread.stop()
