#!/usr/bin/env python
#coding:utf-8
'''
已知R 已知T 已知scale 输出融合里程计
'''

import rospy
from std_msgs.msg import String, UInt32, Float64, Bool
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

velPub = ""
OdomPub = ""
transform = None
lisener=None
odom_combined_tf=None
odom_combined_tf2=None
scale = 0.
prviousCamPose = None
prviousstamp = None
tf_rot=np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
tf_trans=np.array([0.4,0.0,0.])


Odom_last = Odometry()
Odom_last.header.frame_id = "/odom_combined"
Odom_last.child_frame_id = "/base_footprint"
begin_flag=True
globalMovePub=None

camUpdateFlag=False
carUpdateFlag=False

cam_currentPose=Pose()
cam_currentTime=None
car_currentPose=Pose()
car_currentTime=None
cam_lastPose=Pose()

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
        self.wait(4)
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
    car_currentPose.position.z=odom.twist.twist.linear.x
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


def doOdomPub():
    global camUpdateFlag,carUpdateFlag,mStatusLock
    global cam_currentPose,cam_currentTime
    global car_currentPose,car_currentTime
    global car_lastPose,cam_lastPose,Odom_last
    global begin_flag,scale,OdomPub,globalMovePub,velPub
    global tf_rot,tf_trans,scale
    global odom_combined_tf,odom_combined_tf2
    time_now=rospy.Time.now()

    Odom = Odometry()

    Odom.header.stamp = time_now

    Odom.header.frame_id = "/odom_combined"

    mStatusLock.acquire()

    camflag=camUpdateFlag
    carflag=carUpdateFlag

    if begin_flag:
        cam_lastPose=cam_currentPose
        car_lastPose=car_currentPose
    if camUpdateFlag:
        Tad=np.array([cam_currentPose.position.x,cam_currentPose.position.y,cam_currentPose.position.z])
        q=[cam_currentPose.orientation.x,cam_currentPose.orientation.y,cam_currentPose.orientation.z,cam_currentPose.orientation.w]
        M=tf.transformations.quaternion_matrix(q)
        Rad=M[:3,:3]
        #为了简化计算，下文的计算中base_link 和base_footprint被看成是相同的坐标系
        #转到odom_combined，得到camera在odom_combined中的pose
        Rbd=tf_rot.dot(Rad)
        Tbd=scale*(tf_rot.dot(Tad))+tf_trans
        #由camera 的 pose 得到 base_footprint 的pose，这也是下文要发布的pose
        Rdc=tf_rot.T
        Tdc=-1/scale*(Rdc.dot(tf_trans))
        Rbc=Rbd.dot(Rdc)
        Tbc=scale*(Rbd.dot(Tdc))+Tbd

        Odom.pose.pose.position.x =Tbc[0]
        Odom.pose.pose.position.y =Tbc[1]
        Odom.pose.pose.position.z =0. #Tbc[2]
        M=np.identity(4)
        M[:3,:3]=Rbc
        q=tf.transformations.quaternion_from_matrix(M)
        Odom.pose.pose.orientation.x = q[0]
        Odom.pose.pose.orientation.y = q[1]
        Odom.pose.pose.orientation.z = q[2]
        Odom.pose.pose.orientation.w = q[3]
        cam_lastPose=cam_currentPose
    else:
        if carUpdateFlag:

            q=[Odom_last.pose.pose.orientation.x,Odom_last.pose.pose.orientation.y,Odom_last.pose.pose.orientation.z,Odom_last.pose.pose.orientation.w]
            euler=euler_from_quaternion(q)
            theta_last=euler[2]

            delta_car=math.sqrt((car_currentPose.position.x-car_lastPose.position.x)*(car_currentPose.position.x-car_lastPose.position.x)+(car_currentPose.position.y-car_lastPose.position.y)*(car_currentPose.position.y-car_lastPose.position.y))
            if car_currentPose.position.z<0.000001:
                delta_car=-1.*delta_car
            Odom.pose.pose.position.x =Odom_last.pose.pose.position.x+delta_car*math.cos(theta_last)
            Odom.pose.pose.position.y =Odom_last.pose.pose.position.y+delta_car*math.sin(theta_last)
            Odom.pose.pose.position.z =0.


            q1=[car_lastPose.orientation.x,car_lastPose.orientation.y,car_lastPose.orientation.z,car_lastPose.orientation.w]
            q2=[car_currentPose.orientation.x,car_currentPose.orientation.y,car_currentPose.orientation.z,car_currentPose.orientation.w]
            euler1=euler_from_quaternion(q1)
            euler2=euler_from_quaternion(q2)
            theta_dleta=euler2[2]-euler1[2]
            if abs(theta_dleta)>0.4 :
                 theta_dleta=0.0
            q=quaternion_from_euler(0,0,theta_last+theta_dleta)

            Odom.pose.pose.orientation.x = q[0]
            Odom.pose.pose.orientation.y = q[1]
            Odom.pose.pose.orientation.z = q[2]
            Odom.pose.pose.orientation.w = q[3]

        else:
            Odom=Odom_last

        time1_diff=time_now-car_currentTime
        time2_diff=time_now-cam_currentTime
        car_diff=abs(car_currentPose.position.x-car_lastPose.position.x)+abs(car_currentPose.position.y-car_lastPose.position.y)
        if car_diff>2.0 or time2_diff.to_sec()>20. or time1_diff.to_sec()>5.:
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

    if begin_flag:
        Odom_last=Odom
        begin_flag=False

    mStatusLock.release()
    #计算速度
    Odom.child_frame_id = "/base_footprint"
    Odom.twist.twist.linear.x = 30*(Odom.pose.pose.position.x-Odom_last.pose.pose.position.x)
    Odom.twist.twist.linear.y = 30*(Odom.pose.pose.position.y-Odom_last.pose.pose.position.y)

    q1=[Odom_last.pose.pose.orientation.x,Odom_last.pose.pose.orientation.y,Odom_last.pose.pose.orientation.z,Odom_last.pose.pose.orientation.w]
    euler1=euler_from_quaternion(q1)
    q2=[Odom.pose.pose.orientation.x,Odom.pose.pose.orientation.y,Odom.pose.pose.orientation.z,Odom.pose.pose.orientation.w]
    euler2=euler_from_quaternion(q2)
    Odom.twist.twist.angular.z = (euler2[2]-euler1[2])*30.

    OdomPub.publish(Odom)
    q=[Odom.pose.pose.orientation.x,Odom.pose.pose.orientation.y,Odom.pose.pose.orientation.z,Odom.pose.pose.orientation.w]
    M=tf.transformations.quaternion_matrix(q)
    Rbc=M[:3,:3]
    Tbc=np.array([Odom.pose.pose.position.x,Odom.pose.pose.position.y,Odom.pose.pose.position.z])
    if odom_combined_tf != None:
        Rcb=Rbc.T
        T=-Rcb.dot(Tbc)
        M=np.identity(4)
        M[:3,:3]=Rcb
        q=tf.transformations.quaternion_from_matrix(M)
        odom_combined_tf.sendTransform(T,q,
            time_now,
            "/odom_combined",
            "/base_footprint")
    if odom_combined_tf2 != None:
        M=np.identity(4)
        M[:3,:3]=tf_rot
        q=tf.transformations.quaternion_from_matrix(M)
        odom_combined_tf2.sendTransform(tf_trans,q,
            time_now,
            "/ORB_SLAM/World",
            "/odom_combined")
    Odom_last=Odom



def init():
    global orbInitFlag, OdomPub,scale,odom_combined_tf,odom_combined_tf2,orbStartFlag
    global velPub,getTrackThread,globalMovePub
    global cam_currentTime,car_currentTime

    rospy.init_node("odom_combined", anonymous=True)
    cam_currentTime=rospy.Time.now()
    car_currentTime=rospy.Time.now()
    scaleOrbFlag=rospy.get_param('/orb2base_scaleFlag',True)
    if scaleOrbFlag:
        #load scale value from scale.txt
        #file_path=os.path.split(os.path.realpath(__file__))[0]
        #fp3=open("%s/scale.txt"%(file_path),'r+')
        fp3=open("/home/xiaoqiang/slamdb/scale.txt",'r+')
        for line in fp3:
            value_list=line.split(" ")
        scale=float(value_list[0])
        if scale<=0.000001:
            scale=5.
        rospy.set_param('/orb2base_scale',scale)
        fp3.close
        print "scale: "+str(scale)
    odom_combined_tf=tf.TransformBroadcaster()
    odom_combined_tf2=tf.TransformBroadcaster()
    rospy.Subscriber("/system_monitor/report", Status, systemStatusHandler)
    rospy.Subscriber("/ORB_SLAM/Camera", Pose, cameraOdom)
    rospy.Subscriber("/xqserial_server/Odom", Odometry, carOdom)
    globalMovePub = rospy.Publisher('/globalMoveFlag', Bool , queue_size=1)

    velPub = rospy.Publisher('/cmd_vel', Twist , queue_size=5)

    #确保orb 已经启动
    while not orbStartFlag and not rospy.is_shutdown():
	       time.sleep(1)

    #确保orb　已经 track
    time.sleep(2)
    globalMoveFlag=Bool()
    globalMoveFlag.data=True
    globalMovePub.publish(globalMoveFlag)

    while not orbInitFlag and not rospy.is_shutdown():
        if getTrackThread == None:
            getTrackThread = getTrack()
            getTrackThread.start()
            time.sleep(6)
        time.sleep(1)
    if getTrackThread != None:
        getTrackThread.stop()
    OdomPub = rospy.Publisher("/odom_combined", Odometry, queue_size=5)


if __name__ == "__main__":
    init()
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        if OdomPub!=None:
            doOdomPub()
        rate.sleep()
    if getTrackThread != None:
        getTrackThread.stop()
