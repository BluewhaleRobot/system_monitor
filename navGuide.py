#!/usr/bin/env python
# coding: UTF-8
"""
从导航文件读取点的坐标，然后发布给导航程序
"""
import roslib
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import radians, pi
import numpy as  np
import time
import threading
import math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

scale = 1.
tf_rot=np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
tf_trans=np.array([0.4,0.0,0.])

NAV_POINTS_FILE = "/home/xiaoqiang/slamdb/nav.csv"
currentPose=Pose();
mStatusLock = threading.Lock()
poseFlag=False

def getOdom(odom):
    global currentPose,poseFlag
    mStatusLock.acquire()
    if odom != None:
        poseFlag=True
        currentPose=odom.pose.pose; #更新坐标
    else:
        poseFlag=False
    mStatusLock.release()

class MoveBaseSquare:

    def __init__(self):
        global tf_trans,tf_rot,scale,poseFlag,currentPose

        rospy.init_node('nav_guide', anonymous=False)

        rospy.on_shutdown(self.shutdown)

        odomSub=rospy.Subscriber("/odom_combined", Odometry, getOdom)

        self.move_base=None

        navDataFile = open(NAV_POINTS_FILE, "r")
        navDataStr = navDataFile.readline()
        targetPoints = []
        while len(navDataStr) != 0:
            posX = float(navDataStr.split(" ")[0])
            posY = float(navDataStr.split(" ")[1])
            targetPoints.append([posX, posY])
            navDataStr = navDataFile.readline()

        q_angle = quaternion_from_euler(0, 0, 0, axes='sxyz')
        q = Quaternion(*q_angle)



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


        # Create a list to hold the waypoint poses
        # 创建一个列表存储导航点的位置
        waypoints = list()

        # Append each of the four waypoints to the list.  Each waypoint
        # is a pose consisting of a position and orientation in the map frame.
        # 创建四个导航点的位置（角度和坐标位置）
        for point in targetPoints:

            # Tad=np.array(point[0],0.0, point[1])
            # M=tf.transformations.quaternion_matrix(q)
            # Rad=M[:3,:3]
            # #为了简化计算，下文的计算中base_link 和base_footprint被看成是相同的坐标系
            # #转到odom_combined，得到camera在odom_combined中的pose
            # Rbd=tf_rot.dot(Rad)
            # Tbd=scale*(tf_rot.dot(Tad))+tf_trans
            # #由camera 的 pose 得到 base_footprint 的pose，这也是下文要发布的pose
            # Rdc=tf_rot.T
            # Tdc=-1/scale*(Rdc.dot(tf_trans))
            # Rbc=Rbd.dot(Rdc)
            # Tbc=scale*(Rbd.dot(Tdc))+Tbd

            Tad=np.array([point[0],0.0, point[1]])
            Tbc=scale*(tf_rot.dot(Tad))
            waypoints.append(Pose(Point(Tbc[0], Tbc[1], 0.0), q))
        # Initialize the visualization markers for RViz
        # 初始化可视化标记
        self.init_markers()

        mStatusLock.acquire()
        #获取当前机器人坐标系
        while not poseFlag and not rospy.is_shutdown():
            mStatusLock.release()
            time.sleep(1)
            mStatusLock.acquire()
        mStatusLock.release()
        odomSub.unregister()

        max_index=len(waypoints)-1
        mindist_index=0
        mindist=-1;
        curentdist=0;
        # Set a visualization marker at each waypoint
        # 给每个定点的导航点一个可视化标记(就是rviz中看到的粉色圆盘标记)
        i=0
        q2=[currentPose.orientation.x,currentPose.orientation.y,currentPose.orientation.z,currentPose.orientation.w]
        euler2=euler_from_quaternion(q2)
        theta_current=euler2[2]
        for waypoint in waypoints:
            p = Point()
            p = waypoint.position
            self.markers.points.append(p)
            curentdist=(p.x - currentPose.position.x)*(p.x - currentPose.position.x)+(p.y - currentPose.position.y)*(p.y - currentPose.position.y)
            theta_delta=math.atan2(p.y - currentPose.position.y,p.x - currentPose.position.x)
            if theta_delta < 0.:
               theta_delta+=2*3.1415926

            if mindist<0. or (curentdist<mindist and abs(theta_delta-theta_current)<1.6):
                mindist_index=i
                mindist=curentdist
            i=i+1

        # Publisher to manually control the robot (e.g. to stop it)
        # 发布TWist消息控制机器人
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=0)

        # Subscribe to the move_base action server
        # 订阅move_base服务器的消息
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        # Wait 60 seconds for the action server to become available
        # 等待move_base服务器建立
        self.move_base.wait_for_server(rospy.Duration(60))

        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation test")



        while not rospy.is_shutdown():
            # Initialize a counter to track waypoints
            # 初始化一个计数器，记录到达的顶点号
            i = mindist_index
            # Cycle through the four waypoints
            # 主循环,环绕通过四个定点
            while i >=0 and not rospy.is_shutdown():
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


    def move(self, goal):
            # Send the goal pose to the MoveBaseAction server
            # 把目标位置发送给MoveBaseAction的服务器
            self.move_base.send_goal(goal)

            # Allow 1 minute to get there
            # 设定1分钟的时间限制
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(240))

            # If we don't get there in time, abort the goal
            # 如果一分钟之内没有到达，放弃目标
            if not finished_within_time:
                self.move_base.cancel_goal()
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
        marker_lifetime = 0 # 0 is forever
        marker_ns = 'waypoints'
        marker_id = 0
        marker_color = {'r': 1.0, 'g': 0.7, 'b': 1.0, 'a': 1.0}

        # Define a marker publisher.
        # 定义一个标记的发布者
        self.marker_pub = rospy.Publisher('waypoint_markers', Marker, queue_size=0)

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
        if self.move_base !=None:
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
