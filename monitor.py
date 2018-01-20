#!/usr/bin/env python

'''
This is a system monitor node for xiaoqiang. Monitor items are power, odom, brightness etc.
System status will be published at /system_monitor/report
'''
import rospy
from std_msgs.msg import String, UInt32, Float64, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from system_monitor.msg import *
import threading
import os
import commands


reportPub = ""
mStatus = Status()
mStatus.brightness = 0
mStatus.imageStatus = False
mStatus.odomStatus = False
mStatus.orbStartStatus = False
mStatus.orbInitStatus = False
mStatus.power = 0
mStatus.orbScaleStatus = False
powerFlagFilePath = "/home/xiaoqiang/Status/power"
powerLow = 10.0

mStatusLock = threading.Lock()

def getBrightness(brightness):
    mStatusLock.acquire()
    mStatus.brightness = brightness.data
    mStatusLock.release()

def getImage(image):
    mStatusLock.acquire()
    if image != None:
        mStatus.imageStatus = True
    else:
        mStatus.imageStatus = False
    mStatusLock.release()

def getPower(power):
    mStatusLock.acquire()
    mStatus.power = power.data-0.3
    #if power.data < powerLow and power.data > 8.0: #and not os.path.isfile(powerFlagFilePath)
        #flagFile = open(powerFlagFilePath, "w+")
        #flagFile.write(str(power.data))
        #flagFile.close()
        #status, output = commands.getstatusoutput('sudo shutdown -h now')
    mStatusLock.release()

def getOdom(odom):
    mStatusLock.acquire()
    if odom != None:
        mStatus.odomStatus = True
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

def getOrbScaleStatus(flag):
    mStatusLock.acquire()
    mStatus.orbScaleStatus = flag.data
    mStatusLock.release()


def monitor():
    global reportPub
    rospy.init_node("monitor", anonymous=True)
    rospy.Subscriber("/usb_cam/brightness", UInt32, getBrightness)
    rospy.Subscriber("/usb_cam/image_raw", Image, getImage)
    rospy.Subscriber("/ORB_SLAM/Frame", Image, getOrbStartStatus)
    rospy.Subscriber("/xqserial_server/Power", Float64, getPower)
    rospy.Subscriber("/xqserial_server/Odom", Odometry, getOdom)
    rospy.Subscriber("/ORB_SLAM/Camera", rospy.msg.AnyMsg, getOrbTrackingFlag)
    rospy.Subscriber("/orb_scale/scaleStatus", Bool, getOrbScaleStatus)
    reportPub = rospy.Publisher('/system_monitor/report', Status , queue_size
=0)


if __name__ == "__main__":
    monitor()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        mStatusLock.acquire()
        if reportPub !=  "":
            reportPub.publish(mStatus)
        # clear data
        mStatus.brightness = 0
        mStatus.power = 0
        mStatus.imageStatus = False
        mStatus.odomStatus = False
        mStatus.orbInitStatus = False
        mStatus.orbStartStatus = False
        mStatus.orbScaleStatus = False
        mStatusLock.release()
        rate.sleep()
