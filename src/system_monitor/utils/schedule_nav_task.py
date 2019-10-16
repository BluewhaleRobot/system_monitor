# encoding=utf-8

import requests
import json
import time
import threading
import math
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_conjugate
import tf
import rospy


class ScheduleNavTask():

    def __init__(self):
        self.loop_running_flag = False
        self.sleep_time = 0
        self.current_goal_id = -1
        self.current_pose_stamped_map = None
        self.current_pose_stamped = None
        self.running = True
        self.goal_state = "FREE"
        self.status_lock = threading.Lock()
        self.current_goal = None
        self.listener = tf.TransformListener(True, rospy.Duration(10.0))

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
        
        threading.Thread(target=self.update_status).start()

    def update_status(self):
        while self.running:
            res = None
            try:
                res = requests.get(
                    "http://127.0.0.1:3546/api/v1/task?id=current_nav_action")
            except Exception as e:
                print(e)
            if res is not None:
                if res.status_code == 200:
                    task_info = json.loads(res.content.decode("utf-8"))
                    self.current_goal = PoseStamped()
                    self.current_goal.header.frame_id = "map"
                    self.current_goal.header.stamp = rospy.Time.now()
                    self.current_goal.pose.position.x = task_info["x"]
                    self.current_goal.pose.position.y = task_info["y"]
                    self.current_goal.pose.position.z = 0
                    q_angle = quaternion_from_euler(
                        0, 0, task_info["theta"], axes='sxyz')
                    q = Quaternion(*q_angle)
                    self.current_goal.pose.orientation = q
                    self.current_goal_id = -1
                    self.goal_state = task_info["state"]
                else:
                    # 当前无任务
                    self.current_goal = None
                    self.current_goal_id = -1
                    self.goal_state = "FREE"
            time.sleep(1)

    def shutdown(self):
        self.running = False

    def set_goal(self, goal_id):
        pass

    def start_loop(self):
        pass

    def stop_loop(self):
        pass

    def pause(self):
        pass

    def resume(self):
        pass

    def cancel_goal(self):
        pass

    def insert_goal(self, pos_x, pos_y, pos_z, angle=0):
        pass

    def reset_goal(self):
        pass

    def current_goal_status(self):
        return self.goal_state

    def update_pose(self):
        if self.current_pose_stamped == None:
            return -1
        latest = rospy.Time(0)
        self.current_pose_stamped.header.stamp = latest
        try:
            self.current_pose_stamped.header.frame_id = "odom"
            self.current_pose_stamped_map = self.listener.transformPose(
                "map", self.current_pose_stamped)
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException, tf.Exception):
            return -1
        return self.current_pose_stamped_map

    def current_goal_distance(self):
        # get current robot position
        if self.current_goal is None:
            return -1
        if self.current_pose_stamped is None:
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
        return self.pose_distance(self.current_goal.pose, self.current_pose_stamped_map.pose)

    def pose_distance(self, pose1, pose2):
        return math.sqrt(math.pow((pose1.position.x - pose2.position.x), 2)
                         + math.pow((pose1.position.y - pose2.position.y), 2)
                         + math.pow((pose1.position.z - pose2.position.z), 2)
                         )
