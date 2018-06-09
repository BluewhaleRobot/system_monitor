#!/usr/bin/env python
# coding: UTF-8
"""
从导航文件读取点的坐标，然后发布给导航程序
"""
import roslib
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import radians, pi
import numpy as np
import time
import psutil
import subprocess
import signal
import threading
import math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int32, Bool
import tf
from utils.config import TF_ROT,TF_TRANS

tf_rot = TF_ROT
tf_trans = TF_TRANS

NAV_POINTS_FILE = "/home/xiaoqiang/slamdb/nav.csv"
currentPose = Pose()
currentPoseStamped = PoseStamped()
mStatusLock = threading.Lock()
mStatus2Lock = threading.Lock()
poseFlag = False

playFlag = False
playStartTime = None
playFlagTime = None
status = 1
move_base_status = 1


def getOdom(odom):
    global currentPoseStamped, poseFlag
    mStatusLock.acquire()
    if odom != None:
        poseFlag = True
        currentPoseStamped.pose = odom.pose.pose  # 更新坐标
        currentPoseStamped.header = odom.header
    else:
        poseFlag = False
    mStatusLock.release()


def dealCarStatus(carStatu):
    global playFlag, playStartTime, playFlagTime, status

    cmd1 = "aplay /home/xiaoqiang/Desktop/speaker.wav"
    cmd2 = "aplay /home/xiaoqiang/Desktop/engine.wav"

    mStatus2Lock.acquire()
    if move_base_status == 1:
        status = carStatu.data
    else:
        status = 2
    mStatus2Lock.release()

    time_now = rospy.Time.now()
    if status != 2 or playFlagTime == None:
        playFlagTime = time_now

    if playFlag:
        time_diff = time_now - playStartTime
        if time_diff.to_sec() > 8.0:
            playFlag = False

    else:
        if status == 2:
            time_diff = time_now - playFlagTime
            if time_diff.to_sec() > 2.0 and time_diff.to_sec() < 5.0:
                # play speaker
                subprocess.Popen(cmd1, shell=True)
                playStartTime = time_now
                playFlag = True
            elif time_diff.to_sec() > 12.0 and time_diff.to_sec() < 15.0:
                # play engine
                subprocess.Popen(cmd2, shell=True)
                playStartTime = time_now
                playFlag = True
            elif time_diff.to_sec() > 22.0:
                playFlagTime = time_now


def dealCarStatus2(moveBaseStatu):
    global move_base_status
    mStatus2Lock.acquire()
    move_base_status = moveBaseStatu.data
    mStatus2Lock.release()


class MoveBaseSquare:

    def __init__(self):
        global tf_trans, tf_rot, poseFlag, currentPose, currentPoseStamped

        rospy.init_node('nav_guide', anonymous=False)

        rospy.on_shutdown(self.shutdown)

        odomSub = rospy.Subscriber("xqserial_server/Odom", Odometry, getOdom)
        carStatuSub = rospy.Subscriber(
            "/xqserial_server/StatusFlag", Int32, dealCarStatus)
        carStatuSub2 = rospy.Subscriber(
            "/move_base/StatusFlag", Int32, dealCarStatus2)
        self.move_base = None

        navDataFile = open(NAV_POINTS_FILE, "r")
        navDataStr = navDataFile.readline()
        targetPoints = []
        while len(navDataStr) != 0:
            posX = float(navDataStr.split(" ")[0])
            posY = float(navDataStr.split(" ")[1])
            posZ = float(navDataStr.split(" ")[2])
            targetPoints.append([posX, posY, posZ])
            navDataStr = navDataFile.readline()

        q_angle = quaternion_from_euler(0, 0, 0, axes='sxyz')
        q = Quaternion(*q_angle)

        # Create a list to hold the waypoint poses
        # 创建一个列表存储导航点的位置
        waypoints = list()

        # Append each of the four waypoints to the list.  Each waypoint
        # is a pose consisting of a position and orientation in the map frame.
        # 创建四个导航点的位置（角度和坐标位置）
        for point in targetPoints:
            #站点坐标是车体坐标
            Tac = np.array([point[0], point[1], point[2]])
            Tbc = tf_rot.dot(Tac) + tf_trans
            waypoints.append(Pose(Point(Tbc[0], Tbc[1], 0.0), q))
        # Initialize the visualization markers for RViz
        # 初始化可视化标记
        self.init_markers()

        mStatusLock.acquire()

        listener = tf.TransformListener(True, rospy.Duration(10.0))
        tfFlag = False

        while not tfFlag and not rospy.is_shutdown():
            try:
                listener.waitForTransform(
                    "map", "odom", rospy.Time(), rospy.Duration(1.0))
                now = rospy.Time.now()
                listener.waitForTransform(
                    "map", "odom", now, rospy.Duration(1.0))
                tfFlag = True

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception):
                print "oups\n"
                tfFlag = False

        # 获取当前机器人坐标系
        while not poseFlag and not rospy.is_shutdown():
            mStatusLock.release()
            time.sleep(0.1)
            mStatusLock.acquire()
        mStatusLock.release()
        odomSub.unregister()
        # 将机器人坐标系转换成map坐标系
        currentPoseStamped.header.stamp = now
        currentPoseStamped = listener.transformPose("/map", currentPoseStamped)
        currentPose = currentPoseStamped.pose
        max_index = len(waypoints) - 1
        mindist_index = 0
        mindist = -1
        curentdist = 0
        # Set a visualization marker at each waypoint
        # 给每个定点的导航点一个可视化标记(就是rviz中看到的粉色圆盘标记)
        i = 0
        q2 = [currentPose.orientation.x, currentPose.orientation.y,
              currentPose.orientation.z, currentPose.orientation.w]
        euler2 = euler_from_quaternion(q2)
        theta_current = euler2[2]
        # if theta_current < 0.:
        #    theta_current+=2*3.1415926
        # print "theta_current "+str(theta_current)
        # delta_last=0.;
        # delta_current=0.;
        upDirt = True
        if max_index > 0:
            for waypoint in waypoints:
                p = Point()
                p = waypoint.position
                self.markers.points.append(p)
                curentdist = (p.x - currentPose.position.x) * (p.x - currentPose.position.x) + (
                    p.y - currentPose.position.y) * (p.y - currentPose.position.y)
                theta_delta = math.atan2(
                    p.y - currentPose.position.y, p.x - currentPose.position.x)

                # if theta_delta < 0.:
                #    theta_delta+=2*3.1415926
                # print "theta_delta "+str(theta_delta)
                delta_current = abs(theta_delta - theta_current)
                if delta_current > 3.1415926:
                    delta_current = abs(2 * 3.1415926 - delta_current)

                # print "delta_current"+str(delta_current)
                if delta_current < 1.6 and (mindist < 0. or curentdist < mindist):
                    mindist_index = i
                    mindist = curentdist
                    delta_last = delta_current
                    # print "i:"+str(i)
                    if i < max_index:
                        p2 = waypoints[i + 1].position
                    else:
                        p2 = waypoints[0].position

                    if i > 0:
                        p0 = waypoints[i - 1].position
                    else:
                        p0 = waypoints[max_index].position
                    if max_index > 1:
                        theta_next = math.atan2(p2.y - p.y, p2.x - p.x)
                        theta_last = math.atan2(p0.y - p.y, p0.x - p.x)
                        # if theta_next < 0.:
                        #    theta_next+=2*3.1415926
                        # if theta_last < 0.:
                        #    theta_last+=2*3.1415926
                        delta_next0 = abs(theta_next - theta_delta)
                        delta_last0 = abs(theta_last - theta_delta)
                        if delta_next0 > 3.1415926:
                            delta_next0 = 2 * 3.1415926 - delta_next0
                        if delta_last0 > 3.1415926:
                            delta_last0 = 2 * 3.1415926 - delta_last0
                        # print "theta_next "+str(theta_next)
                        # print "theta_last "+str(theta_last)
                        if abs(delta_next0) < abs(delta_last0):
                            upDirt = True
                        else:
                            upDirt = False
                i = i + 1
        print "updirt:" + str(upDirt)
        # while not rospy.is_shutdown():
        #     time.sleep(1)

        # Publisher to manually control the robot (e.g. to stop it)
        # 发布TWist消息控制机器人
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=0)

        # Subscribe to the move_base action server
        # 订阅move_base服务器的消息
        self.move_base = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        # Wait 60 seconds for the action server to become available
        # 等待move_base服务器建立
        self.move_base.wait_for_server(rospy.Duration(60))

        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation test")

        while not rospy.is_shutdown():

            if upDirt:
                # Initialize a counter to track waypoints
                # 初始化一个计数器，记录到达的顶点号
                i = mindist_index
                # Cycle through the four waypoints
                # 主循环,环绕通过四个定点
                while i <= max_index and not rospy.is_shutdown():
                    print i
                    # Update the marker display
                    # 发布标记指示四个目标的位置，每个周期发布一起，确保标记可见
                    # while not rospy.is_shutdown():
                    #     time.sleep(1)
                    #     self.marker_pub.publish(self.markers)
                    #
                    self.marker_pub.publish(self.markers)

                    # Intialize the waypoint goal
                    # 初始化goal为MoveBaseGoal类型
                    goal = MoveBaseGoal()

                    # Use the map frame to define goal poses
                    # 使用map的frame定义goal的frame id
                    goal.target_pose.header.frame_id = 'map'

                    # Set the time stamp to "now"
                    # 设置时间戳
                    goal.target_pose.header.stamp = rospy.Time.now()

                    # Set the goal pose to the i-th waypoint
                    # 设置目标位置是当前第几个导航点
                    goal.target_pose.pose = waypoints[i]

                    # Start the robot moving toward the goal
                    # 机器人移动
                    self.move(goal)

                    i += 1
                    time.sleep(1)

                i = 0
                # Cycle through the four waypoints
                # 主循环,环绕通过四个定点
                while i < mindist_index and not rospy.is_shutdown():
                    print i
                    # Update the marker display
                    # 发布标记指示四个目标的位置，每个周期发布一起，确保标记可见
                    # while not rospy.is_shutdown():
                    #     time.sleep(1)
                    #     self.marker_pub.publish(self.markers)
                    #
                    self.marker_pub.publish(self.markers)

                    # Intialize the waypoint goal
                    # 初始化goal为MoveBaseGoal类型
                    goal = MoveBaseGoal()

                    # Use the map frame to define goal poses
                    # 使用map的frame定义goal的frame id
                    goal.target_pose.header.frame_id = 'map'

                    # Set the time stamp to "now"
                    # 设置时间戳
                    goal.target_pose.header.stamp = rospy.Time.now()

                    # Set the goal pose to the i-th waypoint
                    # 设置目标位置是当前第几个导航点
                    goal.target_pose.pose = waypoints[i]

                    # Start the robot moving toward the goal
                    # 机器人移动
                    self.move(goal)

                    i += 1
                    time.sleep(1)
            else:
                # Initialize a counter to track waypoints
                # 初始化一个计数器，记录到达的顶点号
                i = mindist_index
                # Cycle through the four waypoints
                # 主循环,环绕通过四个定点
                while i >= 0 and not rospy.is_shutdown():
                    print i
                    # Update the marker display
                    # 发布标记指示四个目标的位置，每个周期发布一起，确保标记可见
                    # while not rospy.is_shutdown():
                    #     time.sleep(1)
                    #     self.marker_pub.publish(self.markers)
                    #
                    self.marker_pub.publish(self.markers)

                    # Intialize the waypoint goal
                    # 初始化goal为MoveBaseGoal类型
                    goal = MoveBaseGoal()

                    # Use the map frame to define goal poses
                    # 使用map的frame定义goal的frame id
                    goal.target_pose.header.frame_id = 'map'

                    # Set the time stamp to "now"
                    # 设置时间戳
                    goal.target_pose.header.stamp = rospy.Time.now()

                    # Set the goal pose to the i-th waypoint
                    # 设置目标位置是当前第几个导航点
                    goal.target_pose.pose = waypoints[i]

                    # Start the robot moving toward the goal
                    # 机器人移动
                    self.move(goal)

                    i -= 1
                    time.sleep(1)
                i = max_index
                # Cycle through the four waypoints
                # 主循环,环绕通过四个定点
                while i > mindist_index and not rospy.is_shutdown():
                    print i
                    # Update the marker display
                    # 发布标记指示四个目标的位置，每个周期发布一起，确保标记可见
                    # while not rospy.is_shutdown():
                    #     time.sleep(1)
                    #     self.marker_pub.publish(self.markers)
                    #
                    self.marker_pub.publish(self.markers)

                    # Intialize the waypoint goal
                    # 初始化goal为MoveBaseGoal类型
                    goal = MoveBaseGoal()

                    # Use the map frame to define goal poses
                    # 使用map的frame定义goal的frame id
                    goal.target_pose.header.frame_id = 'map'

                    # Set the time stamp to "now"
                    # 设置时间戳
                    goal.target_pose.header.stamp = rospy.Time.now()

                    # Set the goal pose to the i-th waypoint
                    # 设置目标位置是当前第几个导航点
                    goal.target_pose.pose = waypoints[i]

                    # Start the robot moving toward the goal
                    # 机器人移动
                    self.move(goal)

                    i -= 1
                    time.sleep(1)

    def move(self, goal):
        global status
        # Send the goal pose to the MoveBaseAction server
        # 把目标位置发送给MoveBaseAction的服务器
        self.move_base.send_goal(goal)

        # Allow 1 minute to get there
        # 设定1分钟的时间限制
        finished_within_time = self.move_base.wait_for_result(
            rospy.Duration(880))

        # If we don't get there in time, abort the goal
        # 如果一分钟之内没有到达，放弃目标
        if not finished_within_time:
            self.move_base.cancel_goal()
            if status == 2:
                rospy.loginfo(
                    "Timed out achieving goal,bar dectected ,try again")
                self.move(goal)
            else:
                rospy.loginfo("Timed out achieving goal")
        else:
            # We made it!
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")

    def init_markers(self):
        # Set up our waypoint markers
        # 设置标记的尺寸
        marker_scale = 0.2
        marker_lifetime = 0  # 0 is forever
        marker_ns = 'waypoints'
        marker_id = 0
        marker_color = {'r': 1.0, 'g': 0.7, 'b': 1.0, 'a': 1.0}

        # Define a marker publisher.
        # 定义一个标记的发布者
        self.marker_pub = rospy.Publisher(
            'waypoint_markers', Marker, queue_size=0)

        # Initialize the marker points list.
        # 初始化标记点的列表
        self.markers = Marker()
        self.markers.ns = marker_ns
        self.markers.id = marker_id
        self.markers.type = Marker.CUBE_LIST
        self.markers.action = Marker.ADD
        self.markers.lifetime = rospy.Duration(marker_lifetime)
        self.markers.scale.x = marker_scale
        self.markers.scale.y = marker_scale
        self.markers.color.r = marker_color['r']
        self.markers.color.g = marker_color['g']
        self.markers.color.b = marker_color['b']
        self.markers.color.a = marker_color['a']

        self.markers.header.frame_id = 'map'
        self.markers.header.stamp = rospy.Time.now()
        self.markers.points = list()

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Cancel any active goals
        if self.move_base != None:
            self.move_base.cancel_goal()

        rospy.sleep(2)
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        MoveBaseSquare()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
