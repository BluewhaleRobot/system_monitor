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
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_conjugate
import rosparam

from config import TF_ROT, TF_TRANS
from scipy.spatial.distance import cdist
from scipy import optimize
from nav_msgs.srv import GetPlan, GetPlanRequest, GetMapResponse
from actionlib_msgs.msg import GoalStatus


class NavigationTask():

    def __init__(self, nav_points_file="/home/xiaoqiang/slamdb/nav.csv", nav_path_file="/home/xiaoqiang/slamdb/path.csv"):
        self.nav_points_file = nav_points_file
        self.tf_rot = TF_ROT
        self.tf_trans = TF_TRANS
        self.listener = tf.TransformListener(True, rospy.Duration(10.0))
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=0)
        self.audio_pub = rospy.Publisher(
            '/xiaoqiang_tts/text', String, queue_size=0)
        self.move_base = actionlib.SimpleActionClient("move_base",
                                                      MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server(rospy.Duration(1))
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation test")
        self.current_pose_stamped = None
        self.current_pose_stamped_map = None
        self.status_lock = threading.Lock()
        self.current_goal_id = -1
        self.goal_status = "FREE"
        self.loop_running_flag = False
        self.loop_exited_flag = True
        self.sleep_time = 1
        self.last_speed = None
        self.load_targets_exited_flag = True
        self.running_flag = True
        self.goal_lock = threading.Lock()
        self.original_target_points = None
        self.original_waypoints = None

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
            self.last_speed = msg
            if self.goal_status == "PAUSED":
                return
            self.cmd_vel_pub.publish(msg)

        self.nav_cmd_vel_sub = rospy.Subscriber(
            "/cmd_vel_nav", Twist, send_cmd_vel)

        self.start_load_targets()

    def load_targets_task(self):
        with self.goal_lock:
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

            self.waypoints = list()
            for point in self.target_points:
                pose_in_world = PoseStamped()
                pose_in_world.header.frame_id = "map"
                pose_in_world.header.stamp = rospy.Time(0)
                pose_in_world.pose.position = Point(point[0], point[1], point[2])
                q_angle = quaternion_from_euler(
                    0, 0, 0, axes='sxyz')
                pose_in_world.pose.orientation = Quaternion(*q_angle)
                self.waypoints.append(pose_in_world)
            self.load_targets_exited_flag = True
            # 通过全局规划器，计算目标点的朝向
            rospy.loginfo("waiting for move_base/make_plan service")
            rospy.wait_for_service("/move_base/make_plan")
            rospy.loginfo("waiting for move_base/make_plan service succeed")
            make_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
            for waypoint in self.waypoints:
                req = GetPlanRequest()
                req.start = self.waypoints[0]
                if waypoint == self.waypoints[0]:
                    rospy.loginfo("Set 0 point direction")
                    req.start = self.waypoints[1]
                    req.goal = self.waypoints[0]  # 修复0号点方向问题
                else:
                    req.goal = waypoint
                req.tolerance = 0.1
                res = None
                try:
                    res = make_plan(req)
                except Exception as e:
                    rospy.logerr(e)
                    time.sleep(2)
                    # 再次尝试调用
                    try:
                        res = make_plan(req)
                    except Exception as ex:
                        rospy.logerr(ex)
                if res == None:
                    # 获取角度失败
                    continue
                # 截断，优化速度
                res.plan.poses = res.plan.poses[-10:]
                plan_path_2d = [[point.pose.position.x, point.pose.position.y]
                                for point in res.plan.poses]
                if len(plan_path_2d) < 4:
                    rospy.logwarn("Not enough point to calculate direction")
                    continue
                angle = self.get_target_direction(
                    [waypoint.pose.position.x, waypoint.pose.position.y], plan_path_2d)
                if waypoint == self.waypoints[0]:
                    # 0号点头朝向1号点
                    q_angle = quaternion_from_euler(0, 0, math.atan2(
                        angle[1], angle[0]), axes='sxyz')
                else:
                    q_angle = quaternion_from_euler(0, 0, math.atan2(
                        angle[1], angle[0]) + math.pi, axes='sxyz')
                waypoint.pose.orientation = Quaternion(*q_angle)
            self.original_target_points = list(self.target_points)
            self.original_waypoints = list(self.waypoints)
            rosparam.set_param("/galileo/goal_num", str(len(self.target_points)))

    def start_load_targets(self):
        if self.load_targets_exited_flag:
            self.load_targets_exited_flag = False
            threading._start_new_thread(self.load_targets_task, ())

    def shutdown(self):
        rospy.loginfo("shutdown called")
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
        self.running_flag = False
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
        self.goal_status = "WORKING"
        goal = MoveBaseGoal()
        goal.target_pose = self.waypoints[goal_id]

        def done_cb(status, result):
            self.goal_status = "FREE"
            # if status == GoalStatus.SUCCEEDED:
            #     target_names = ["A", "B", "C", "D", "E", "F", "G"]
            #     target_desp = ["蓝鲸智能机器人为您服务", "等待乘客上车，十秒后出发",
            #                    "正在加油，十秒后加油完成，完成后出发", "感谢乘坐本次班车，蓝鲸智能机器人为您服务", "", "", "", "", "", ""]
            #     if goal_id < len(target_names):
            #         self.audio_pub.publish("到达{name}号目标点，{target_desp}"
            #                            .format(name=target_names[goal_id], target_desp=target_desp[goal_id]))
        # wait for 1s
        if not self.move_base.wait_for_server(rospy.Duration(1)):
            self.goal_status = "ERROR"
            self.cancel_goal()
            return
        self.move_base.send_goal(goal, done_cb=done_cb)

    def pause(self):
        if self.current_goal_status() == "WORKING":
            self.goal_status = "PAUSED"
        if self.last_speed != None:
            current_speed = self.last_speed.linear.x
            for i in range(0, 100):
                speed = Twist()
                speed.linear.x = current_speed * (100 - i) / 100.0
                self.cmd_vel_pub.publish(speed)
                time.sleep(0.01)
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
        self.current_pose_stamped.header.frame_id = "odom"
        try:
            self.current_pose_stamped_map = self.listener.transformPose(
                "/map", self.current_pose_stamped)
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException, tf.Exception):
            return -1
        mgoal = self.waypoints[self.current_goal_id].pose
        return self.pose_distance(mgoal, self.current_pose_stamped_map.pose)

    def update_pose(self):
        latest = rospy.Time(0)
        self.current_pose_stamped.header.stamp = latest
        try:
            self.current_pose_stamped.header.frame_id = "odom"
            self.current_pose_stamped_map = self.listener.transformPose(
                "/map", self.current_pose_stamped)
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException, tf.Exception):
            return -1
        return self.current_pose_stamped_map

    def pose_distance(self, pose1, pose2):
        return math.sqrt(math.pow((pose1.position.x - pose2.position.x), 2)
                         + math.pow((pose1.position.y - pose2.position.y), 2)
                         + math.pow((pose1.position.z - pose2.position.z), 2)
                         )

    # 插入点坐标为map坐标系下
    def insert_goal(self, pos_x, pos_y, pos_z, angle=0):
        with self.goal_lock:
            # 插入一个新点
            q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
            q = Quaternion(*q_angle)
            waypoint = Pose(Point(pos_x, pos_y, pos_z), q)
            waypoint_stamped = PoseStamped()
            waypoint_stamped.header.frame_id = "map"
            waypoint_stamped.header.stamp = rospy.Time(0)
            waypoint_stamped.pose = waypoint
            self.waypoints.append(waypoint_stamped)
            self.target_points.append(waypoint_stamped)
            rosparam.set_param("/galileo/goal_num", str(len(self.target_points)))

    def reset_goals(self):
        self.cancel_goal()
        with self.goal_lock:
            self.current_goal_id = -1
            self.goal_status = "FREE"
            self.target_points = list(self.original_target_points)
            self.waypoints = list(self.original_waypoints)
            rosparam.set_param("/galileo/goal_num", str(len(self.target_points)))

    def loop_task(self):
        # 获取当前最近的位置
        self.loop_exited_flag = False
        rospy.loginfo("获取TF")
        tf_flag = False
        self.listener = tf.TransformListener(True, rospy.Duration(10.0))
        while not tf_flag and not rospy.is_shutdown() and self.running_flag:
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
        if not self.running_flag:
            self.loop_exited_flag = True
            return
        rospy.loginfo("获取TF 成功")
        self.current_pose_stamped.header.stamp = now
        if not tf_flag:
            return
        with self.status_lock:
            self.current_pose_stamped_map = self.listener.transformPose(
                "/map", self.current_pose_stamped)
        current_pose = self.current_pose_stamped_map.pose
        current_pose_q = [current_pose.orientation.x, current_pose.orientation.y,
                          current_pose.orientation.z, current_pose.orientation.w]
        theta_current = euler_from_quaternion(current_pose_q)[2]
        rospy.loginfo("current_pose: " + str(current_pose))
        rospy.loginfo("计算发布点")
        # 找到第一个要走的目标点,在机器人正面且距离最近的点
        next_target = None
        mini_distance = 0
        # waypoints 是在map坐标系的点
        back_points = []  # 在机器人背后的点
        for point in [waypoint.pose for waypoint in self.waypoints]:
            # 判断是否在机器人前方
            theta_delta = math.atan2(
                point.position.y - current_pose.position.y, point.position.x - current_pose.position.x)
            delta_current = abs(theta_delta - theta_current)
            if delta_current > 3.1415926:
                delta_current = abs(2 * 3.1415926 - delta_current)

            rospy.loginfo("delta_current: " + str(delta_current))
            rospy.loginfo("current pose: " + str(current_pose.position.x) +
                          " " + str(current_pose.position.y))
            rospy.loginfo("point: " + str(point.position.x) +
                          " " + str(point.position.y))
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

        next_index = [waypoint.pose for waypoint in self.waypoints].index(
            next_target)
        while not rospy.is_shutdown() and self.loop_running_flag and self.running_flag:
            rospy.loginfo("next goal " + str(next_index))
            self.set_goal(next_index)

            # 设置导航点失败，可能由于系统尚未初始化
            # 未处于工作状态，且未处于任务完成状态
            while self.goal_status != "WORKING" and not \
                (self.goal_status == "FREE" and self.current_goal_distance() < 0.2
                    and self.current_goal_distance() > 0):
                time.sleep(1)
                self.set_goal(next_index)

            while not rospy.is_shutdown() and self.loop_running_flag and self.running_flag:
                if self.goal_status == "FREE":
                    break
                if not self.loop_running_flag:
                    self.loop_exited_flag = True
                    return
                time.sleep(0.5)
            if not self.running_flag:
                self.loop_exited_flag = True
                return

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

    def get_target_direction(self, target_point, nav_path_points):
        # 映射至二维点，首先在ORB_SLAM/World坐标系下进行计算
        target_point_2d = target_point
        nav_path_points_2d = nav_path_points
        # 获取距此目标点最近的路径点
        nearest_point = self.closest_node(target_point_2d, nav_path_points_2d)
        # 找距离此路径点最近的其他路径点
        nav_path_points_2d_filterd = filter(
            lambda point: point[0] != target_point_2d[0] or point[1] != target_point_2d[1], nav_path_points_2d)
        nearest_point_2 = self.closest_node(
            nearest_point, nav_path_points_2d_filterd)
        # 距此路径点第二近点
        nav_path_points_2d_filterd = filter(
            lambda point: point[0] != nearest_point_2[0] or point[1] != nearest_point_2[1], nav_path_points_2d_filterd)
        nearest_point_3 = self.closest_node(
            nearest_point, nav_path_points_2d_filterd)

        def f_1(x, A, B):
            return A*x + B
        A1, _ = optimize.curve_fit(f_1, [nearest_point[0], nearest_point_2[0], nearest_point_3[0]],
                                   [nearest_point[1], nearest_point_2[1], nearest_point_3[1]])[0]
        if (nearest_point_3[0] - nearest_point[0]) * A1 >= 0:
            return (1 / A1, 1)
        else:
            return (-1 / A1, -1)

    def closest_node(self, node, nodes):
        filtered_nodes = filter(
            lambda point: point[0] != node[0] or point[1] != node[1], nodes)
        return filtered_nodes[cdist([node], filtered_nodes).argmin()]

    def set_angle(self, angle):
        goal = MoveBaseGoal()
        target_pose = PoseStamped()
        target_pose.pose.position = self.current_pose_stamped_map.pose.position
        q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
        target_pose.pose.orientation = Quaternion(*q_angle)
        goal.target_pose = target_pose
        def done_cb(status, result):
            self.goal_status = "FREE"
        # wait for 1s
        if not self.move_base.wait_for_server(rospy.Duration(1)):
            self.goal_status = "ERROR"
            return
        self.move_base.send_goal(goal, done_cb=done_cb)
