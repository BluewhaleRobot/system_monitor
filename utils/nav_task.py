#!/usr/bin/env python
# coding:utf-8

import rospy
from std_msgs.msg import String, UInt32, Float64, Bool, Int16
from nav_msgs.msg import Odometry
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Image
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
from visualization_msgs.msg import Marker
import math
from math import radians, pi
import numpy as np


class NavTask():

    """docstring for NavTask"""

    def __init__(self, NAV_POINTS_FILE="/home/xiaoqiang/slamdb/nav.csv"):
        self.NAV_POINTS_FILE = NAV_POINTS_FILE
        self.tf_rot = np.array([[0., 0.03818382, 0.99927073],
                                [-1., 0., 0.], [0., -0.99927073, 0.03818382]])
        self.tf_trans = np.array([0.0, 0.0, 0.])
        self.load_targets()
        self.init_markers()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=0)
        self.move_base = actionlib.SimpleActionClient("move_base",
                                                      MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server(rospy.Duration(1))
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation test")
        self.currentPoseStamped = None
        self.status_lock = threading.Lock()
        self.listener = tf.TransformListener(True, rospy.Duration(10.0))
        self.current_goal_id = -1
        self.goal_status = "FREE"

        def get_odom(odom):
            with self.status_lock:
                if odom != None:
                    self.currentPoseStamped = PoseStamped()
                    self.currentPoseStamped.pose = odom.pose.pose
                    self.currentPoseStamped.header = odom.header
                else:
                    self.currentPoseStamped = None

        self.odom_sub = rospy.Subscriber(
            "xqserial_server/Odom", Odometry, get_odom)

        def send_cmd_vel(msg):
            if self.goal_status == "PAUSED":
                return
            self.cmd_vel_pub.publish(msg)

        self.nav_cmd_vel_sub = rospy.Subscriber(
            "/cmd_vel_nav", Twist, send_cmd_vel)

    def load_targets(self):
        with open(self.NAV_POINTS_FILE, "r") as nav_data_file:
            nav_data_str = nav_data_file.readline()
            self.target_points = []
            while len(nav_data_str) != 0:
                pos_x = float(nav_data_str.split(" ")[0])
                pos_y = float(nav_data_str.split(" ")[1])
                pos_z = float(nav_data_str.split(" ")[2])
                self.target_points.append([pos_x, pos_y, pos_z])
                nav_data_str = nav_data_file.readline()

        q_angle = quaternion_from_euler(0, 0, 0, axes='sxyz')
        q = Quaternion(*q_angle)

        value_list = []
        with open("/home/xiaoqiang/slamdb/scale.txt", "r+") as scale_file:
            for line in scale_file:
                value_list = line.split(" ")
        scale = float(value_list[0])
        if scale <= 0.000001:
            scale = 5.
        rospy.set_param('/orb2base_scale', scale)
        print("scale: " + str(scale))
        self.waypoints = list()
        for point in self.target_points:
            Tad = np.array([point[0], point[1], point[2]])
            Tbc = scale * (self.tf_rot.dot(Tad)) + self.tf_trans
            self.waypoints.append(Pose(Point(Tbc[0], Tbc[1], 0.0), q))

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
        self.odom_sub.unregister()
        self.nav_cmd_vel_sub.unregister()
        rospy.sleep(2)
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

    def set_goal(self, goal_id):
        if self.current_goal_status() != "FREE":
            self.cancel_goal()
        # invalid goal id
        if goal_id > len(self.waypoints):
            self.goal_status = "ERROR"
            return
        self.current_goal_id = goal_id
        self.marker_pub.publish(self.markers)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.waypoints[goal_id]
        def done_cb(status, result):
            self.goal_status = "FREE"
        # wait for 1s
        if not self.move_base.wait_for_server(rospy.Duration(1)):
            self.goal_status = "ERROR"
            self.cancel_goal()
            return
        self.move_base.send_goal(goal, done_cb=done_cb)
        self.goal_status = "WORKING"

    def pause(self):
        if self.current_goal_status() == "WORKING":
            self.goal_status = "PAUSED"
        self.cmd_vel_pub.publish(Twist())

    def resume(self):
        if self.current_goal_status() == "PAUSED":
            self.goal_status = "WORKING"

    def cancel_goal(self):
        if self.current_goal_status() != "FREE":
            self.move_base.cancel_goal()
            self.cmd_vel_pub.publish(Twist())
        self.goal_status = "FREE"

    """
    target status related
    """

    def current_goal(self):
        if self.current_goal_id != -1:
            return self.target_points[self.current_goal_id]
        return None

    def current_goal_status(self):
        return self.goal_status

    def current_goal_distance(self):
        # get current robot position
        if self.current_goal_id == -1:
            return -1
        if self.currentPoseStamped == None:
            return -1

        latest = rospy.Time(0)
        self.currentPoseStamped.header.stamp = latest
        try:
            self.currentPoseStamped = self.listener.transformPose(
                "/map", self.currentPoseStamped)
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException, tf.Exception):
            return -1
        currentPose = self.currentPoseStamped.pose
        mgoal = self.waypoints[self.current_goal_id]
        return self.pose_distance(mgoal, currentPose)

    def pose_distance(self, pose1, pose2):
        return math.sqrt(math.pow((pose1.position.x - pose2.position.x), 2)
                         + math.pow((pose1.position.y - pose2.position.y), 2)
                         + math.pow((pose1.position.z - pose2.position.z), 2)
                         )
