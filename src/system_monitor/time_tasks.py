#!/usr/bin/env python
#encoding=utf-8

import rospy
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool, String
from galileo_serial_server.msg import GalileoNativeCmds, GalileoStatus
import time
import rosservice
import subprocess
import os
import schedule
import json
from datetime import datetime
import shutil
import threading
import schedule
import requests
import hashlib
import threading

MAPS_DB_PATH = "/home/xiaoqiang/saved-slamdb"
CURRENT_DB_PATH = "/home/xiaoqiang/slamdb"

NAV_STATUS = 0
TASKFILE_STATUS = 0

TASK_RUNING = 0

STATUS_LOCK = threading.Lock()
FILESTATUS_LOCK = threading.Lock()

AUDIO_PUB = None

GALILEO_PUB = None

TASK_LIST = list()

POWER_LOW = 37

POWER_NOW = 0

CHARGE_FLAG = 0

POWER_RECORDS = []

class FileChangeHandler():
    def __init__(self):
        global MAPS_DB_PATH
        self.md5_last_value = None
        if os.path.exists(MAPS_DB_PATH+"/timeTask.json"):
            taskfile = open(MAPS_DB_PATH+"/timeTask.json",'rb')
            self.md5_last_value = hashlib.md5(taskfile.read()).hexdigest()
            taskfile.close()
    def if_changed(self):
        global FILESTATUS_LOCK,TASKFILE_STATUS,MAPS_DB_PATH
        with FILESTATUS_LOCK:
            self.md5_value = None
            TASKFILE_STATUS = 0
            if os.path.exists(MAPS_DB_PATH+"/timeTask.json"):
                taskfile = open(MAPS_DB_PATH+"/timeTask.json",'rb')
                self.md5_value = hashlib.md5(taskfile.read()).hexdigest()
                taskfile.close()
            if self.md5_value == self.md5_last_value:
                TASKFILE_STATUS = 0
                return False
            else:
                self.md5_last_value = self.md5_value
                TASKFILE_STATUS = 1
                return True


def status_update_cb(status):
    global NAV_STATUS,STATUS_LOCK,POWER_NOW,CHARGE_FLAG,POWER_RECORDS
    with STATUS_LOCK:
        NAV_STATUS = status.navStatus
        if status.power > 0 :
            if len(POWER_RECORDS) < 30 * 5:
                POWER_RECORDS.append(status.power)
            else:
                POWER_RECORDS.pop(0)
                POWER_RECORDS.append(status.power)
            POWER_NOW = sum(POWER_RECORDS) / len(POWER_RECORDS);
        CHARGE_FLAG = status.chargeStatus

def change_map(map_name, path_name):
    global MAPS_DB_PATH, CURRENT_DB_PATH
    rospy.set_param("/system_monitor/nav_is_enabled", False)
    #先拷贝地图
    map_src_path =  MAPS_DB_PATH + "/" + map_name

    if map_name == "":
        rospy.set_param("/system_monitor/nav_is_enabled", True)
        return False

    if path_name == "":
        rospy.set_param("/system_monitor/nav_is_enabled", True)
        return False

    if not os.path.exists(map_src_path):
        rospy.set_param("/system_monitor/nav_is_enabled", True)
        return False

    #如果nav_check 或者 path_check 失败，则不切换
    nav_check = os.path.exists(map_src_path + "/nav_" + path_name + ".csv")
    path_check = os.path.exists(map_src_path + "/path_" + path_name + ".csv")
    if not nav_check or not path_check:
        rospy.set_param("/system_monitor/nav_is_enabled", True)
        return False

    if os.path.exists(CURRENT_DB_PATH):
        shutil.rmtree(CURRENT_DB_PATH)
    shutil.copytree(map_src_path, CURRENT_DB_PATH)

    #拷贝地图路线
    nav_src = CURRENT_DB_PATH + "/nav_" + path_name + ".csv"
    new_nav_src = CURRENT_DB_PATH + "/new_nav_" + path_name + ".csv"
    path_src = CURRENT_DB_PATH + "/path_" + path_name + ".csv"

    nav_dst = CURRENT_DB_PATH + "/nav.csv"
    new_nav_dst = CURRENT_DB_PATH + "/new_nav.csv"
    path_dst  = CURRENT_DB_PATH + "/path.csv"


    if os.path.exists(nav_src):
        shutil.copy(nav_src, nav_dst)
    if os.path.exists(new_nav_src):
        shutil.copy(new_nav_src, new_nav_dst)
    if os.path.exists(path_src):
        shutil.copy(path_src, path_dst)

    rospy.set_param("/system_monitor/nav_is_enabled", True)
    return True



class MapSwitchTask():
    def __init__(self, whichDay, startTime, stopTime, mapName, pathName):
        self.whichDay = whichDay
        self.startTime = startTime
        self.stopTime = stopTime
        self.mapName = mapName
        self.pathName = pathName

        dt_obj = datetime.strptime(self.startTime, "%H:%M")
        self.startTime_value = dt_obj.hour*60 + dt_obj.minute

        dt_obj2 = datetime.strptime(self.stopTime, "%H:%M")
        self.stopTime_value = dt_obj2.hour*60 + dt_obj2.minute

    def ifneed_runnow(self):
        time_now = datetime.now()
        time_now_value = time_now.hour*60 + time_now.minute
        #print "now "+ str(time_now_value) + " task " + str(self.startTime_value)
        #print "daynow "+ str(time_now.day) + " daytask " + str(self.whichDay)
        if self.whichDay == 0 or self.whichDay == (time_now.weekday() + 1):
            if time_now_value >= self.startTime_value and time_now_value<self.stopTime_value:
                return True

        return False

    def job_needdone(self):
        global NAV_STATUS,AUDIO_PUB,GALILEO_PUB,STATUS_LOCK,TASK_RUNING,TASK_NEEDSTOP,POWER_NOW,CHARGE_FLAG
        #先等待上一个任务执行完成
        STATUS_LOCK.acquire()
        last_task_status = TASK_RUNING
        STATUS_LOCK.release()
        while last_task_status != 0 and self.ifneed_runnow():
            time.sleep(3)
            STATUS_LOCK.acquire()
            last_task_status = TASK_RUNING
            TASK_NEEDSTOP = 1
            STATUS_LOCK.release()

        # #如果正在充电同时电池电压低于设定值，先充电，直到满足条件
        # STATUS_LOCK.acquire()
        # charge_flag_now = CHARGE_FLAG
        # power_now_value = POWER_NOW
        # STATUS_LOCK.release()
        # while charge_flag_now == 1 and self.ifneed_runnow():
        #     if power_now_value > POWER_LOW:
        #         #停止充电
        #         galileo_cmds = GalileoNativeCmds()
        #         galileo_cmds.data = 'j' + chr(0x01)
        #         galileo_cmds.length = len(galileo_cmds.data)
        #         galileo_cmds.header.stamp = rospy.Time.now()
        #         GALILEO_PUB.publish(galileo_cmds)
        #     time.sleep(3)
        #     STATUS_LOCK.acquire()
        #     charge_flag_now = CHARGE_FLAG
        #     power_now_value = POWER_NOW
        #     STATUS_LOCK.release()

        if not self.ifneed_runnow():
            return

        STATUS_LOCK.acquire()
        TASK_RUNING = 1
        TASK_NEEDSTOP = 0
        STATUS_LOCK.release()

        AUDIO_PUB.publish("开始切换地图")
        #切换地图过程，发布导航服务开启禁用话题
        STATUS_LOCK.acquire()
        nav_status = NAV_STATUS
        STATUS_LOCK.release()

        if change_map(self.mapName, self.pathName):
            if nav_status != 1:
                AUDIO_PUB.publish("地图切换成功")
                STATUS_LOCK.acquire()
                TASK_RUNING = 0
                STATUS_LOCK.release()
                return

            #todo 地图切换完成后要把正在进行的导航任务进行重置
            #先关闭
            rospy.set_param("/system_monitor/nav_is_enabled", False)
            AUDIO_PUB.publish("地图切换成功, 开始重新载入导航任务")
            galileo_cmds = GalileoNativeCmds()
            galileo_cmds.data = 'm' + chr(0x04)
            galileo_cmds.length = len(galileo_cmds.data)
            galileo_cmds.header.stamp = rospy.Time.now()
            max_do = 0
            task_needstop_now = 0
            while nav_status == 1 and task_needstop_now == 0:
                GALILEO_PUB.publish(galileo_cmds)
                time.sleep(3)
                STATUS_LOCK.acquire()
                nav_status = NAV_STATUS
                task_needstop_now = TASK_NEEDSTOP
                STATUS_LOCK.release()
                max_do = max_do +1;
                if max_do >20:
                    break
            #再重新开启
            rospy.set_param("/system_monitor/nav_is_enabled", True)
            nav_status = 0
            galileo_cmds.data = 'm' + chr(0x00)
            galileo_cmds.length = len(galileo_cmds.data)
            galileo_cmds.header.stamp = rospy.Time.now()
            max_do = 0
            task_needstop_now = 0
            while nav_status != 1 and task_needstop_now == 0:
                GALILEO_PUB.publish(galileo_cmds)
                time.sleep(30)
                STATUS_LOCK.acquire()
                nav_status = NAV_STATUS
                task_needstop_now = TASK_NEEDSTOP
                STATUS_LOCK.release()
                max_do = max_do +1;
                if max_do >10:
                    break
            STATUS_LOCK.acquire()
            TASK_RUNING = 0
            STATUS_LOCK.release()
        else:
            AUDIO_PUB.publish("地图切换失败")
            STATUS_LOCK.acquire()
            TASK_RUNING = 0
            STATUS_LOCK.release()

class AutoRunTask():
    def __init__(self, whichDay, startTime, stopTime, mapName, pathName, waitTime, stopPose, loopCount):
        self.whichDay = whichDay
        self.startTime = startTime
        self.stopTime = stopTime
        self.mapName = mapName
        self.pathName = pathName
        self.waitTime = waitTime
        self.stopPose = stopPose
        self.loopCount = loopCount

        dt_obj = datetime.strptime(self.startTime, "%H:%M")
        self.startTime_value = dt_obj.hour*60 + dt_obj.minute

        dt_obj2 = datetime.strptime(self.stopTime, "%H:%M")
        self.stopTime_value = dt_obj2.hour*60 + dt_obj2.minute

        self.new_nav_points_file="/home/xiaoqiang/slamdb/new_nav.csv"
        self.waypoints = []
        self.looptaskid = None
        self.looptaskflag = False
        self.loopCountlast = 0

    def ifneed_runnow(self):
        time_now = datetime.now()
        time_now_value = time_now.hour*60 + time_now.minute
        #print "now "+ str(time_now_value) + " task " + str(self.startTime_value)
        #print "daynow "+ str(time_now.day) + " daytask " + str(self.whichDay)
        if self.whichDay == 0 or self.whichDay == (time_now.weekday() + 1):
            if time_now_value >= self.startTime_value and time_now_value<self.stopTime_value:
                return True
        return False

    def ifloopCount_over(self):
        if self.looptaskid != None:
            try:
                payload = {"id": self.looptaskid}
                r = requests.get('http://127.0.0.1:3546/api/v1/task', params=payload, timeout=5)
                if r.status_code == requests.codes.ok:
                    task_details = r.json()
                    self.looptask_state = task_details["state"]

                    if self.loopCount >0 and task_details["loop_count"] >= (self.loopCount - self.loopCountlast):
                        self.looptaskflag = True
                        self.loopCountlast = task_details["loop_count"]
                        return True

                    if task_details["state"] == "CANCELLED" or  task_details["state"] == "ERROR" or task_details["state"] == "COMPLETE":
                        if task_details["state"] == "COMPLETE":
                            self.looptaskflag = True

                        self.loopCountlast = task_details["loop_count"]
                        return True

                return False
            except Exception:
                return True
        return True

    def ifneed_stopnow(self):
        if self.ifloopCount_over():
            return True
        if not self.ifneed_runnow():
            self.looptaskflag = True
            return True
        return False

    def load_point(self):
        plan_mode = 0
        self.waypoints = list()
        with open(self.new_nav_points_file, "r") as new_nav_data_file:
            new_nav_data_str = new_nav_data_file.readline()
            while len(new_nav_data_str) != 0:
                #print(len(self.waypoints))
                pos_x = float(new_nav_data_str.split(" ")[0])
                pos_y = float(new_nav_data_str.split(" ")[1])
                pos_z = float(new_nav_data_str.split(" ")[2])
                angle = float(new_nav_data_str.split(" ")[3])
                self.waypoints.append([pos_x, pos_y, angle])
                new_nav_data_str = new_nav_data_file.readline()
                if len(new_nav_data_str.split(" ")) == 7:
                    plan_mode = int(new_nav_data_str.split(" ")[6])
        rospy.set_param("/NLlinepatrol_planner/ab_direction", plan_mode)

        self.sub_actions = list()
        for point in self.waypoints:
            self.sub_actions.append({"type": "nav_action", "x": point[0], "y": point[1], "theta": point[2] })
            self.sub_actions.append({"type": "sleep_action", "wait_time": self.waitTime })

    def doloop_task(self):
        global NAV_STATUS,AUDIO_PUB,GALILEO_PUB,STATUS_LOCK,TASK_RUNING,TASK_NEEDSTOP,POWER_NOW,CHARGE_FLAG
        self.looptaskflag = False
        self.looptask_state = "WAITTING"
        self.looptask_active = False
        try:
            AUDIO_PUB.publish("开始循环")
            if len(self.waypoints) > 0:
                payload = { "name": "doloop_task",
                            "sub_tasks": self.sub_actions,
                            "loop_flag": True
                          }
                r = requests.post('http://127.0.0.1:3546/api/v1/task', json=payload, timeout=5)
                if r.status_code == requests.codes.ok:
                    task_details = r.json()
                    self.looptaskid = task_details["id"]

                    payload2 = {'id': self.looptaskid}
                    r2 = requests.get('http://127.0.0.1:3546/api/v1/task/start', params=payload2, timeout=5)
                    print(self.looptaskid)
                    self.looptask_active = True

            #等待任务完成
            STATUS_LOCK.acquire()
            task_needstop_now = TASK_NEEDSTOP
            charge_flag_now = CHARGE_FLAG
            nav_status = NAV_STATUS
            STATUS_LOCK.release()
            needstop = False
            while self.looptaskid != None and task_needstop_now == 0 and self.ifneed_runnow() and nav_status == 1 and not self.looptaskflag:
                #每秒查询一次结果
                time.sleep(1)
                if self.looptask_active:
                    needstop = self.ifneed_stopnow()

                STATUS_LOCK.acquire()
                task_needstop_now = TASK_NEEDSTOP
                charge_flag_now = CHARGE_FLAG
                nav_status = NAV_STATUS
                STATUS_LOCK.release()

                if rospy.is_shutdown() or ((charge_flag_now == 1 or task_needstop_now == 1 or needstop) and self.looptask_active):
                    #取消任务
                    payload = {'id': self.looptaskid}
                    r = requests.get('http://127.0.0.1:3546/api/v1/task/stop', params=payload, timeout=5)
                    self.looptask_active = False
                    self.looptask_state = "CANCELLED"
                    AUDIO_PUB.publish("暂停循环")

                if rospy.is_shutdown():
                    break

                if self.looptask_state == "CANCELLED" and nav_status == 1 and charge_flag_now ==2 and not self.looptaskflag:
                    #停止充电
                    galileo_cmds = GalileoNativeCmds()
                    galileo_cmds.data = 'j' + chr(0x01)
                    galileo_cmds.length = len(galileo_cmds.data)
                    galileo_cmds.header.stamp = rospy.Time.now()
                    GALILEO_PUB.publish(galileo_cmds)
                    #重启任务
                    time.sleep(3)
                    if len(self.waypoints) > 0:
                        payload3 = {'id': self.looptaskid}

                        r3 = requests.get('http://127.0.0.1:3546/api/v1/task/start', params=payload3, timeout=5)
                        if r3.status_code == requests.codes.ok:
                            task_details = r.json()
                            self.looptaskid = task_details["id"]
                            print(self.looptaskid)
                            self.looptask_state = "WAITTING"
                            self.looptask_active = True
                            AUDIO_PUB.publish("继续循环")
            AUDIO_PUB.publish("结束循环")
        except Exception as e:
            print(e)
            return

    def dostop_task(self):
        global NAV_STATUS,AUDIO_PUB,GALILEO_PUB,STATUS_LOCK,TASK_RUNING,TASK_NEEDSTOP,POWER_NOW,CHARGE_FLAG
        taskid = None
        try:
            if self.stopPose <0:
                #回去充电
                r = requests.get('http://127.0.0.1:3546/api/v1/navigation/go_charge', timeout=5)
                if r.status_code == requests.codes.ok:
                    task_details = r.json()
                    taskid = task_details["id"]
            else:
                #回到指定目标位置
                goal_id = self.stopPose
                if goal_id < len(self.waypoints):
                    point = self.waypoints[goal_id]
                    payload = { "name": "stoppose_task",
                                "sub_tasks": [
                                {"type": "nav_action", "x": point[0], "y": point[1], "theta": point[2] },
                                ],
                              }
                    r = requests.post('http://127.0.0.1:3546/api/v1/task', json=payload, timeout=5)
                    if r.status_code == requests.codes.ok:
                        task_details = r.json()
                        taskid = task_details["id"]

                        payload2 = {'id': taskid}
                        r2 = requests.get('http://127.0.0.1:3546/api/v1/task/start', params=payload2, timeout=5)
                        print(taskid)

            #等待任务完成
            task_needstop_now = 0
            while taskid != None and task_needstop_now == 0 and not rospy.is_shutdown():
                #每秒查询一次结果
                time.sleep(1)
                payload = {'id': taskid}
                r = requests.get('http://127.0.0.1:3546/api/v1/task', params=payload, timeout=5)
                if r.status_code == requests.codes.ok:
                    task_details = r.json()
                    if task_details["state"] == "CANCELLED" or  task_details["state"] == "ERROR" or task_details["state"] == "COMPLETE":
                        break
                STATUS_LOCK.acquire()
                task_needstop_now = TASK_NEEDSTOP
                STATUS_LOCK.release()
                if task_needstop_now == 1:
                    #取消任务
                    r = requests.get('http://127.0.0.1:3546/api/v1/task/stop', params=payload, timeout=5)
        except Exception as e:
            print(e)
            return

        #停止导航
        rospy.set_param("/system_monitor/nav_is_enabled", False)
        galileo_cmds = GalileoNativeCmds()
        galileo_cmds.data = 'm' + chr(0x04)
        galileo_cmds.length = len(galileo_cmds.data)
        galileo_cmds.header.stamp = rospy.Time.now()
        max_do = 0
        STATUS_LOCK.acquire()
        nav_status = NAV_STATUS
        task_needstop_now = TASK_NEEDSTOP
        STATUS_LOCK.release()
        while nav_status == 1 and task_needstop_now == 0:
            GALILEO_PUB.publish(galileo_cmds)
            time.sleep(10)
            STATUS_LOCK.acquire()
            nav_status = NAV_STATUS
            task_needstop_now = TASK_NEEDSTOP
            STATUS_LOCK.release()
            max_do = max_do +1;
            if max_do >20:
                break
        rospy.set_param("/system_monitor/nav_is_enabled", True)

    def job_needdone(self):
        global NAV_STATUS,AUDIO_PUB,GALILEO_PUB,STATUS_LOCK,TASK_RUNING,TASK_NEEDSTOP,POWER_LOW,POWER_NOW,CHARGE_FLAG
        #先等待上一个任务执行完成
        STATUS_LOCK.acquire()
        last_task_status = TASK_RUNING
        STATUS_LOCK.release()
        while last_task_status != 0 and self.ifneed_runnow() and not rospy.is_shutdown():
            time.sleep(3)
            STATUS_LOCK.acquire()
            last_task_status = TASK_RUNING
            TASK_NEEDSTOP = 1
            STATUS_LOCK.release()

        #如果正在充电同时电池电压低于设定值，先充电，直到满足条件
        STATUS_LOCK.acquire()
        charge_flag_now = CHARGE_FLAG
        power_now_value = POWER_NOW
        STATUS_LOCK.release()
        while charge_flag_now == 1 and self.ifneed_runnow() and not rospy.is_shutdown():
            if power_now_value > POWER_LOW:
                #停止充电
                galileo_cmds = GalileoNativeCmds()
                galileo_cmds.data = 'j' + chr(0x01)
                galileo_cmds.length = len(galileo_cmds.data)
                galileo_cmds.header.stamp = rospy.Time.now()
                GALILEO_PUB.publish(galileo_cmds)
            time.sleep(3)
            STATUS_LOCK.acquire()
            charge_flag_now = CHARGE_FLAG
            power_now_value = POWER_NOW
            STATUS_LOCK.release()

        while charge_flag_now == 2 and self.ifneed_runnow() and not rospy.is_shutdown():
            #停止充电
            galileo_cmds = GalileoNativeCmds()
            galileo_cmds.data = 'j' + chr(0x01)
            galileo_cmds.length = len(galileo_cmds.data)
            galileo_cmds.header.stamp = rospy.Time.now()
            GALILEO_PUB.publish(galileo_cmds)
            time.sleep(3)
            STATUS_LOCK.acquire()
            charge_flag_now = CHARGE_FLAG
            power_now_value = POWER_NOW
            STATUS_LOCK.release()

        if not self.ifneed_runnow():
            return

        if rospy.is_shutdown():
            return

        STATUS_LOCK.acquire()
        TASK_RUNING = 1
        TASK_NEEDSTOP = 0
        nav_status = NAV_STATUS
        STATUS_LOCK.release()

        AUDIO_PUB.publish("尝试开始定时循环任务")

        #第一步先切换地图
        if change_map(self.mapName, self.pathName):
            #第二步关闭现有导航任务
            #发布导航服务开启禁用话题
            try:
                r = requests.get('http://127.0.0.1:3546/api/v1/task/stop', timeout=5)
            except Exception as e:
                print(e)
                return
            rospy.set_param("/system_monitor/nav_is_enabled", False)
            AUDIO_PUB.publish("地图切换成功, 开始重新载入导航任务")
            galileo_cmds = GalileoNativeCmds()
            galileo_cmds.data = 'm' + chr(0x04)
            galileo_cmds.length = len(galileo_cmds.data)
            galileo_cmds.header.stamp = rospy.Time.now()
            max_do = 0
            task_needstop_now = 0
            while nav_status == 1 and task_needstop_now == 0:
                GALILEO_PUB.publish(galileo_cmds)
                try:
                    r = requests.get('http://127.0.0.1:3546/api/v1/task/stop', timeout=5)
                except Exception as e:
                    print(e)
                    return
                time.sleep(10)
                STATUS_LOCK.acquire()
                task_needstop_now = TASK_NEEDSTOP
                nav_status = NAV_STATUS
                STATUS_LOCK.release()
                max_do = max_do +1;
                if max_do >20:
                    break
            #再重新开启
            rospy.set_param("/system_monitor/nav_is_enabled", True)
            #第二步开始循环任务
            nav_status = 0
            galileo_cmds.data = 'm' + chr(0x00)
            galileo_cmds.length = len(galileo_cmds.data)
            galileo_cmds.header.stamp = rospy.Time.now()
            max_do = 0
            task_needstop_now = 0
            while nav_status != 1 and task_needstop_now == 0 :
                GALILEO_PUB.publish(galileo_cmds)
                time.sleep(10)
                STATUS_LOCK.acquire()
                nav_status = NAV_STATUS
                task_needstop_now = TASK_NEEDSTOP
                STATUS_LOCK.release()

            self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

            rospy.loginfo("Waiting for move_base action server...")
            # 等待move_base服务器建立
            serverlag = False
            task_needstop_now = 0
            while not serverlag and task_needstop_now == 0 :
                serverlag = self.move_base.wait_for_server(rospy.Duration(10))
                STATUS_LOCK.acquire()
                task_needstop_now = TASK_NEEDSTOP
                STATUS_LOCK.release()

            rospy.logerror("Connected to move base server")

            if nav_status == 1 and task_needstop_now == 0 and not rospy.is_shutdown():
                self.load_point()
                self.doloop_task()
                STATUS_LOCK.acquire()
                task_needstop_now = TASK_NEEDSTOP
                STATUS_LOCK.release()
                if task_needstop_now == 0 and self.looptaskflag and not rospy.is_shutdown():
                    self.dostop_task()
            STATUS_LOCK.acquire()
            TASK_RUNING = 0
            STATUS_LOCK.release()
        else:
            AUDIO_PUB.publish("地图切换失败，无法开启定时循环任务")
            STATUS_LOCK.acquire()
            TASK_RUNING = 0
            STATUS_LOCK.release()

def job_test(test_str):
    global AUDIO_PUB
    AUDIO_PUB.publish(test_str)


def load_tasks():
    global MAPS_DB_PATH, CURRENT_DB_PATH, TASK_LIST
    TASK_LIST = list()
    schedule.clear()
    if not os.path.exists(MAPS_DB_PATH+"/timeTask.json"):
        return False

    index_task = 0
    with open(MAPS_DB_PATH+"/timeTask.json") as json_file:
        list_jobj = json.load(json_file)
        for task in list_jobj["mapSwithTasks"]:
            print(task)
            TASK_LIST.append(MapSwitchTask(task["whichDay"],task["startTime"], task["stopTime"], task["mapName"], task["pathName"]))

            if task["whichDay"] == 0:
                #schedule.every().day.at(task["startTime"]).do(job_test,"测试")
                schedule.every().day.at(task["startTime"]).do(TASK_LIST[index_task].job_needdone)
            elif task["whichDay"] == 1:
                schedule.every().monday.at(task["startTime"]).do(TASK_LIST[index_task].job_needdone)
            elif task["whichDay"] == 2:
                schedule.every().tuesday.at(task["startTime"]).do(TASK_LIST[index_task].job_needdone)
            elif task["whichDay"] == 3:
                schedule.every().wednesday.at(task["startTime"]).do(TASK_LIST[index_task].job_needdone)
            elif task["whichDay"] == 4:
                schedule.every().thursday.at(task["startTime"]).do(TASK_LIST[index_task].job_needdone)
            elif task["whichDay"] == 5:
                schedule.every().friday.at(task["startTime"]).do(TASK_LIST[index_task].job_needdone)
            elif task["whichDay"] == 6:
                schedule.every().saturday.at(task["startTime"]).do(TASK_LIST[index_task].job_needdone)
            elif task["whichDay"] == 7:
                schedule.every().sunday.at(task["startTime"]).do(TASK_LIST[index_task].job_needdone)
            index_task = index_task + 1

        for task in list_jobj["autoRunTasks"]:
            print(task)
            TASK_LIST.append(AutoRunTask(task["whichDay"], task["startTime"], task["stopTime"], task["mapName"], task["pathName"], task["waitTime"], task["stopPose"], task["loopCount"]))

            if task["whichDay"] == 0:
                #schedule.every().day.at(task["startTime"]).do(job_test,"测试")
                schedule.every().day.at(task["startTime"]).do(TASK_LIST[index_task].job_needdone)
            elif task["whichDay"] == 1:
                schedule.every().monday.at(task["startTime"]).do(TASK_LIST[index_task].job_needdone)
            elif task["whichDay"] == 2:
                schedule.every().tuesday.at(task["startTime"]).do(TASK_LIST[index_task].job_needdone)
            elif task["whichDay"] == 3:
                schedule.every().wednesday.at(task["startTime"]).do(TASK_LIST[index_task].job_needdone)
            elif task["whichDay"] == 4:
                schedule.every().thursday.at(task["startTime"]).do(TASK_LIST[index_task].job_needdone)
            elif task["whichDay"] == 5:
                schedule.every().friday.at(task["startTime"]).do(TASK_LIST[index_task].job_needdone)
            elif task["whichDay"] == 6:
                schedule.every().saturday.at(task["startTime"]).do(TASK_LIST[index_task].job_needdone)
            elif task["whichDay"] == 7:
                schedule.every().sunday.at(task["startTime"]).do(TASK_LIST[index_task].job_needdone)
            index_task = index_task + 1

        index_task = 0
        for job_now in schedule.jobs:
            if TASK_LIST[index_task].ifneed_runnow() :
                #任务需要立即运行
                #print "runnow " + str(index_task)
                threading._start_new_thread(job_now.run, ())
            index_task = index_task + 1

if __name__ == "__main__":

    rospy.init_node("time_task_server")

    AUDIO_PUB = rospy.Publisher("/xiaoqiang_tts/text", String, queue_size=10)

    GALILEO_PUB = rospy.Publisher("/galileo/cmds", GalileoNativeCmds, queue_size=5)

    status_sub = rospy.Subscriber("/galileo/status", GalileoStatus, status_update_cb)

    POWER_LOW = float(rospy.get_param("~power_low", "37.0"))
    #开始从json文件中加载地图切换任务
    time.sleep(60) #先等待开机1分钟
    load_tasks()
    index_i = 0

    filechange_handler = FileChangeHandler()

    while not rospy.is_shutdown():
        #任务最多每分钟执行一次
        schedule.run_pending()
        #每10分钟检查一下任务是否要重新载入
        if index_i >=3:
            index_i = 0
            filechange_handler.if_changed()
            FILESTATUS_LOCK.acquire()
            if TASKFILE_STATUS == 1:
                 TASKFILE_STATUS = 0
                 load_tasks()
            FILESTATUS_LOCK.release()

        index_i = index_i +1
        time.sleep(60)
