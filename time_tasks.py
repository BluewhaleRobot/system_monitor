#!/usr/bin/env python
#encoding=utf-8

import rospy
from std_msgs.msg import Bool, String
from galileo_serial_server.msg import GalileoNativeCmds, GalileoStatus
import time
import rosservice
import subprocess
import os
import schedule
import json
from datetime import datetime
from shutil import copyfile
import threading
import schedule
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

MAPS_DB_PATH = "/home/xiaoqiang/saved-slamdb"
CURRENT_DB_PATH = "/home/xiaoqiang/slamdb"

NAV_STATUS = 0
TASKFILE_STATUS = 0

STATUS_LOCK = threading.Lock()
FILESTATUS_LOCK = threading.Lock()

AUDIO_PUB = None

GALILEO_PUB = None

TASK_LIST = list()


class FileChangeHandler(FileSystemEventHandler):
    global FILESTATUS_LOCK,TASKFILE_STATUS,MAPS_DB_PATH
    def on_modified(self, event):
        with FILESTATUS_LOCK:
            taskfile_path = MAPS_DB_PATH+"/timeTask.json"
            if event.src_path == taskfile_path:
                TASKFILE_STATUS = 1
                print(f'event type: {event.event_type}  path : {event.src_path}')

def status_update_cb(status):
    global NAV_STATUS
    with STATUS_LOCK:
        NAV_STATUS = status.navStatus

def change_map(map_name, path_name):
    global MAPS_DB_PATH, CURRENT_DB_PATH

    #先拷贝地图
    map_src_path =  MAPS_DB_PATH + "/" + map_name

    if map_name == "":
        return False

    if path_name == "":
        return False

    if not os.path.exists(map_src_path):
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
    path_dst = = CURRENT_DB_PATH + "/path.csv"

    shutil.copy(nav_src, nav_dst)
    shutil.copy(new_nav_src, new_nav_dst)
    shutil.copy(path_nav_src, path_nav_dst)

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

        if self.whichDay == 0 or self.whichDay == time_now.day:
            if time_now_value >= self.startTime_value and time_now_value<self.stopTime_value:
                return True

        return False

    def job_needdone(self):
        global NAV_STATUS,AUDIO_PUB，GALILEO_PUB
            AUDIO_PUB.publish("开始切换地图")
            #切换地图过程，发布导航服务开启禁用话题
            STATUS_LOCK.acquire()
            nav_status = NAV_STATUS
            STATUS_LOCK.release()

            if change_map(self.mapName, self.pathName):
                if nav_status != 1:
                    AUDIO_PUB.publish("地图切换成功")
                    return

                #todo 地图切换完成后要把正在进行的导航任务进行重置
                #先关闭
                AUDIO_PUB.publish("地图切换成功, 开始重新载入导航任务")
                galileo_cmds.data = 'm' + chr(0x04)
                galileo_cmds.length = len(galileo_cmds.data)
                galileo_cmds.header.stamp = rospy.Time.now()
                GALILEO_PUB.publish(galileo_cmds)
                max_do = 0
                while nav_status == 1:
                    time.sleep(3)
                    STATUS_LOCK.acquire()
                    nav_status = NAV_STATUS
                    STATUS_LOCK.release()
                    max_do = max_do +1;
                    if max_do >20:
                        break
                #再重新开启
                nav_status = 0
                galileo_cmds.data = 'm' + chr(0x00)
                galileo_cmds.length = len(galileo_cmds.data)
                galileo_cmds.header.stamp = rospy.Time.now()
                GALILEO_PUB.publish(galileo_cmds)
                max_do = 0
                while nav_status != 1:
                    time.sleep(3)
                    STATUS_LOCK.acquire()
                    nav_status = NAV_STATUS
                    STATUS_LOCK.release()
                    max_do = max_do +1;
                    if max_do >20:
                        break
                return
            else:
                AUDIO_PUB.publish("地图切换失败")


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
                schedule.every().day.at(task["startTime"]+":00").do(TASK_LIST[index_task].job_needdone()).tag("mapSwithTask",str(index_task))
            elif task.whichDay == 1:
                schedule.every().monday.at(task["startTime"]+":00").do(TASK_LIST[index_task].job_needdone()).tag("mapSwithTask",str(index_task))
            elif task.whichDay == 2:
                schedule.every().tuesday.at(task["startTime"]+":00").do(TASK_LIST[index_task].job_needdone()).tag("mapSwithTask",str(index_task))
            elif task.whichDay == 3:
                schedule.every().wednesday.at(task["startTime"]+":00").do(TASK_LIST[index_task].job_needdone()).tag("mapSwithTask",str(index_task))
            elif task.whichDay == 4:
                schedule.every().thursday.at(task["startTime"]+":00").do(TASK_LIST[index_task].job_needdone()).tag("mapSwithTask",str(index_task))
            elif task.whichDay == 5:
                schedule.every().friday.at(task["startTime"]+":00").do(TASK_LIST[index_task].job_needdone()).tag("mapSwithTask",str(index_task))
            elif task.whichDay == 6:
                schedule.every().saturday.at(task["startTime"]+":00").do(TASK_LIST[index_task].job_needdone()).tag("mapSwithTask",str(index_task))
            elif task.whichDay == 7:
                schedule.every().sunday.at(task["startTime"]+":00").do(TASK_LIST[index_task].job_needdone()).tag("mapSwithTask",str(index_task))
            index_task = index_task + 1

        index_task = 0
        for job_now in schedule.jobs:
            if TASK_LIST[index_task].ifneed_runnow() :
                #任务需要立即运行
                job_now.run()

if __name__ == "__main__":
    rospy.init_node("status_audio_reporter")
    AUDIO_PUB = rospy.Publisher("/xiaoqiang_tts/text", String, queue_size=10)

    GALILEO_PUB = rospy.Publisher("/galileo/cmds", GalileoNativeCmds, queue_size=5)

    status_sub = rospy.Subscriber("/galileo/status", GalileoStatus, status_update_cb)

    #开始从json文件中加载地图切换任务
    load_tasks()
    index_i = 0

    event_handler = FileChangeHandler()
    observer = Observer()
    observer.schedule(event_handler, path=MAPS_DB_PATH, recursive=False)
    observer.start()

    while not rospy.is_shutdown():
        #任务最多每分钟执行一次
        schedule.run_pending()
        #每10分钟检查一下任务是否要重新载入
        if index_i >10:
            index_i = 0
            FILESTATUS_LOCK.acquire()
            if TASKFILE_STATUS == 1:
                 TASKFILE_STATUS = 0
                 load_tasks()
            FILESTATUS_LOCK.release()

        index_i = index_i +1
        time.sleep(60)
    observer.stop()
    observer.join()
