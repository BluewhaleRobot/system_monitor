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
# Author: Randoms, Xiefusheng
#

import math
import threading
import time
import os

import actionlib
import numpy as np
import rospy
import tf
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64, Int16, String, UInt32
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from config import TF_ROT, TF_TRANS


class NavigationTask():

    def __init__(self, nav_points_file="/home/xiaoqiang/slamdb/nav.csv"):
        self.nav_points_file = nav_points_file
        self.tf_rot = TF_ROT
        self.tf_trans = TF_TRANS
        self.load_targets()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=0)
        self.move_base = actionlib.SimpleActionClient("move_base",
                                                      MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server(rospy.Duration(1))
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation test")
        self.current_pose_stamped = None
        self.status_lock = threading.Lock()
        self.listener = tf.TransformListener(True, rospy.Duration(10.0))
        self.current_goal_id = -1
        self.goal_status = "FREE"
        self.loop_running_flag = False
        self.loop_exited_flag = True
        self.sleep_time = 1
        self.track_init_flag = False

        def get_odom(odom):
            with self.status_lock:
                if odom != None:
                    self.current_pose_stamped = PoseStamped()
                    self.current_pose_stamped.pose = odom.pose.pose
                    self.current_pose_stamped.header = odom.header
                else:
                    self.current_pose_stamped = None

        self.odom_sub = rospy.Subscriber(
            "xqserial_server/Odom", Odometry, get_odom)

        def send_cmd_vel(msg):
            if self.goal_status == "PAUSED":
                return
            self.cmd_vel_pub.publish(msg)

        self.nav_cmd_vel_sub = rospy.Subscriber(
            "/cmd_vel_nav", Twist, send_cmd_vel)

    def load_targets(self):
        if not os.path.exists(self.nav_points_file):
            self.target_points = []
            self.waypoints = list()

        with open(self.nav_points_file, "r") as nav_data_file:
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

        self.waypoints = list()
        for point in self.target_points:
            Tac = np.array([point[0], point[1], point[2]])
            Tbc = self.tf_rot.dot(Tac) + self.tf_trans
            self.waypoints.append(Pose(Point(Tbc[0], Tbc[1], 0.0), q))

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.stop_loop()
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
        if goal_id >= len(self.waypoints):
            self.goal_status = "ERROR"
            rospy.logerr("Invalid goal id: " + str(goal_id))
            return
        self.current_goal_id = goal_id
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
        self.current_goal_id = -1
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
        if self.current_pose_stamped == None:
            return -1

        latest = rospy.Time(0)
        self.current_pose_stamped.header.stamp = latest
        try:
            self.current_pose_stamped = self.listener.transformPose(
                "/map", self.current_pose_stamped)
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException, tf.Exception):
            return -1
        current_pose = self.current_pose_stamped.pose
        mgoal = self.waypoints[self.current_goal_id]
        return self.pose_distance(mgoal, current_pose)

    def pose_distance(self, pose1, pose2):
        return math.sqrt(math.pow((pose1.position.x - pose2.position.x), 2)
                         + math.pow((pose1.position.y - pose2.position.y), 2)
                         + math.pow((pose1.position.z - pose2.position.z), 2)
                         )

    # 插入点坐标为map坐标系下
    def insert_goal(self, pos_x, pos_y, pos_z):
        # 插入一个新点
        # target_points在ORB_SLAM/World坐标系下
        q_angle = quaternion_from_euler(0, 0, 0, axes='sxyz')
        q = Quaternion(*q_angle)
        waypoint = Pose(Point(pos_x, pos_y, pos_z), q)
        self.waypoints.append(waypoint)
        waypoint_stamped = PoseStamped()
        waypoint_stamped.header.frame_id = "map"
        waypoint_stamped.header.stamp = rospy.Time(0)
        waypoint_stamped.pose = waypoint

        # 转至ORB_SLAM/World坐标系
        rospy.loginfo("获取TF map->ORB_SLAM/World")
        tf_flag = False
        while not tf_flag and not rospy.is_shutdown():
            try:
                t = rospy.Time(0);
                self.listener.waitForTransform("map", "ORB_SLAM/World", t,
                                               rospy.Duration(1.0))
                tf_flag = True
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException, tf.Exception) as e:
                tf_flag = False
                rospy.logwarn("获取TF失败 map->ORB_SLAM/World")
                rospy.logwarn(e)
        rospy.loginfo("获取TF 成功 map->ORB_SLAM/World")
        target_point = self.listener.transformPose(
            "ORB_SLAM/World", waypoint_stamped)
        target_point.header.stamp = rospy.Time.now()
        self.target_points.append(target_point)


    def reset_goals(self):
        self.current_goal_id = -1
        self.goal_status = "FREE"
        self.load_targets()

    def loop_task(self):
        # 获取当前最近的位置
        self.loop_exited_flag = False
        rospy.loginfo("获取TF")
        tf_flag = False
        self.listener = tf.TransformListener(True, rospy.Duration(10.0))
        while not tf_flag and not rospy.is_shutdown():
            try:
                now = rospy.Time.now()
                self.listener.waitForTransform("map", "odom", now,
                                               rospy.Duration(1.0))
                tf_flag = True
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException, tf.Exception) as e:
                tf_flag = False
                rospy.logwarn("获取TF失败")
                rospy.logwarn(e)
        rospy.loginfo("获取TF 成功")
        self.current_pose_stamped.header.stamp = now
        if not tf_flag:
            return
        with self.status_lock:
            self.current_pose_stamped = self.listener.transformPose(
                "/map", self.current_pose_stamped)
        current_pose = self.current_pose_stamped.pose
        current_pose_q = [current_pose.orientation.x, current_pose.orientation.y,
                          current_pose.orientation.z, current_pose.orientation.w]
        theta_current = euler_from_quaternion(current_pose_q)[2]
        rospy.loginfo("current_pose: " + str(current_pose))
        rospy.loginfo("计算发布点")
        # 找到第一个要走的目标点,在机器人正面且距离最近的点
        next_target = None
        mini_distance = 0
        # waypoints 是在map坐标系的点
        back_points = [] # 在机器人背后的点
        for point in self.waypoints:
            # 判断是否在机器人前方
            theta_delta = math.atan2(
                point.position.y - current_pose.position.y, point.position.x - current_pose.position.x)
            delta_current = abs(theta_delta - theta_current)
            if delta_current > 3.1415926:
                delta_current = abs(2 * 3.1415926 - delta_current)

            rospy.loginfo("delta_current: " + str(delta_current))
            rospy.loginfo("current pose: " + str(current_pose.position.x) + " " + str(current_pose.position.y))
            rospy.loginfo("point: " + str(point.position.x) + " " + str(point.position.y))
            rospy.loginfo("theta_current: " + str(theta_current))
            rospy.loginfo("theta_delta: " + str(theta_delta))
            if delta_current > math.pi / 2:
                back_points.append(point)
                continue
            current_distance = self.pose_distance(current_pose, point)

            if current_distance < mini_distance or mini_distance == 0:
                mini_distance = current_distance
                next_target = point
        # 如果正面没有目标点，则选择最近的目标点
        mini_distance = 0
        if next_target is None:
            for point in back_points:
                current_distance = self.pose_distance(current_pose, point)
                if current_distance < mini_distance or mini_distance == 0:
                    mini_distance = current_distance
                    next_target = point

        next_index = self.waypoints.index(next_target)
        while not rospy.is_shutdown() and self.loop_running_flag:
            rospy.loginfo("next goal " + str(next_index))
            self.set_goal(next_index)
            # 等待goal的状态
            time.sleep(1)
            while not rospy.is_shutdown():
                if self.goal_status == "FREE" and \
                        self.current_goal_distance() < 0.2 and \
                        self.current_goal_distance() > 0:
                    break
                if not self.loop_running_flag:
                    self.loop_exited_flag = True
                    return
                time.sleep(0.5)
            next_index += 1
            next_index = next_index % len(self.waypoints)
            sleep_count = 0
            while sleep_count < self.sleep_time:
                time.sleep(0.01)
                sleep_count += 0.01
                if not self.loop_running_flag:
                    self.loop_exited_flag = True
                    return
        self.loop_exited_flag = True


    def start_loop(self):
        self.loop_running_flag = True
        if self.loop_exited_flag:
            threading._start_new_thread(self.loop_task, ())

    def stop_loop(self):
        self.loop_running_flag = False
