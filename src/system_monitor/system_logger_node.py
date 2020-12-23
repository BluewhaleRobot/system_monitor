#!/usr/bin/env python
#encoding=utf-8

import rospy
from galileo_serial_server.msg import GalileoStatus
from xiaoqiang_log.msg import LogRecord
import time
import json
import os
import hashlib

previous_status = None
log_pub = None
work_stamp_start = 0
pause_stamp_start = 0

test = GalileoStatus()
test.header.stamp.to_nsec() / 1000 / 1000
task_record = {
    "events": [],
    "results": ""
}

def get_current_map_info():
    MAP_WORKSPACE_PATH = "/home/xiaoqiang/slamdb"
    if not os.path.exists(os.path.join(MAP_WORKSPACE_PATH, "map_name.txt")):
        return None
    # 获取地图名称
    map_name = ""
    with open(os.path.join(MAP_WORKSPACE_PATH, "map_name.txt"), 'r') as map_name_file:
        map_name = map_name_file.read().strip()
    # 计算路径文件md5值
    path_name = ""
    if not os.path.exists(os.path.join(MAP_WORKSPACE_PATH, "nav.csv")):
        return {
            "map_name": map_name,
            "path_name": path_name
        }
    with open(os.path.join(MAP_WORKSPACE_PATH, "nav.csv"), 'rb') as nav_file:
        data = nav_file.read()
    nav_file_md5= hashlib.md5(data).hexdigest()
    map_files = os.listdir(MAP_WORKSPACE_PATH)
    for map_file in map_files:
        if map_file.startswith("nav_"):
            with open(os.path.join(MAP_WORKSPACE_PATH, map_file), 'rb') as path_file:
                data = path_file.read()
                path_file_md5 = hashlib.md5(data).hexdigest()
                if path_file_md5 == nav_file_md5:
                    return {
                        "map_name": map_name,
                        "path_name": map_file[4:-4]
                    }
    return {
        "map_name": map_name,
        "path_name": path_name
    }

def galileo_status_logger(status):
    global previous_status, work_stamp_start, pause_stamp_start, log_pub, task_record
    if previous_status is None:
        previous_status = status
        return
    # 从free状态到work状态
    if previous_status.targetStatus == 0 and status.targetStatus == 1:
        work_stamp_start = status.header.stamp.to_nsec() / 1000 / 1000 # ms单位时间戳
    
    current_stamp = status.header.stamp.to_nsec() / 1000 / 1000

    # 从work状态到free状态,正常完成后targetNumID不为,如果取消任务则为-1
    if previous_status.targetStatus == 1 and status.targetStatus == 0 and status.targetNumID != -1:
        current_map_info = get_current_map_info()
        task_record["events"].append({
            "type": "WORKING",
            "duration": current_stamp - work_stamp_start,
            "timestamp": int(time.time() * 1000),
            "goal_index": status.targetNumID,
            "map": "" if current_map_info is None else current_map_info["map_name"],
            "path": "" if current_map_info is None else current_map_info["path_name"]
        })
        task_record["results"] = "SUCCESS"

        # pub log
        log_record = LogRecord()
        log_record.stamp = status.header.stamp
        log_record.collection_name = "task"
        log_record.record = json.dumps(task_record)
        log_pub.publish(log_record)

        # reset data
        work_stamp_start = 0
        pause_stamp_start = 0
        task_record = {
            "events": [],
            "results": ""
        }

        rospy.loginfo("从work状态到free状态")
    
    # 从working状态到pause状态
    if previous_status.targetStatus == 1 and status.targetStatus == 2:
        current_map_info = get_current_map_info()
        task_record["events"].append({
            "type": "WORKING",
            "duration": current_stamp - work_stamp_start,
            "timestamp": int(time.time() * 1000),
            "goal_index": status.targetNumID,
            "map": "" if current_map_info is None else current_map_info["map_name"],
            "path": "" if current_map_info is None else current_map_info["path_name"]
        })
        pause_stamp_start = current_stamp
        work_stamp_start = 0

    # 从pause状态到working状态
    if previous_status.targetStatus == 2 and status.targetStatus == 1:
        current_map_info = get_current_map_info()
        task_record["events"].append({
            "type": "PAUSED",
            "duration": current_stamp - pause_stamp_start,
            "timestamp": int(time.time() * 1000),
            "goal_index": status.targetNumID,
            "map": "" if current_map_info is None else current_map_info["map_name"],
            "path": "" if current_map_info is None else current_map_info["path_name"]
        })
        work_stamp_start = current_stamp
        pause_stamp_start = 0

    # 从working状态到cancel状态
    if previous_status.targetStatus == 1 and status.targetStatus == 0 and status.targetNumID == -1:
        current_map_info = get_current_map_info()
        task_record["events"].append({
            "type": "WORKING",
            "duration": current_stamp - work_stamp_start,
            "timestamp": int(time.time() * 1000),
            "goal_index": status.targetNumID,
            "map": "" if current_map_info is None else current_map_info["map_name"],
            "path": "" if current_map_info is None else current_map_info["path_name"]
        })
        task_record["results"] = "CANCEL"

        # pub log
        log_record = LogRecord()
        log_record.stamp = status.header.stamp
        log_record.collection_name = "task"
        log_record.record = json.dumps(task_record)
        log_pub.publish(log_record)

        # reset data
        work_stamp_start = 0
        pause_stamp_start = 0
        task_record = {
            "events": [],
            "results": ""
        }
        rospy.loginfo("从working状态到cancel状态")

    # 从pause状态到cancel状态
    if previous_status.targetStatus == 2 and status.targetStatus == 0:
        current_map_info = get_current_map_info()
        task_record["events"].append({
            "type": "PAUSED",
            "duration": current_stamp - pause_stamp_start,
            "timestamp": int(time.time() * 1000),
            "goal_index": status.targetNumID,
            "map": "" if current_map_info is None else current_map_info["map_name"],
            "path": "" if current_map_info is None else current_map_info["path_name"]
        })
        task_record["results"] = "CANCEL"

        # pub log
        log_record = LogRecord()
        log_record.stamp = status.header.stamp
        log_record.collection_name = "task"
        log_record.record = json.dumps(task_record)
        log_pub.publish(log_record)

        # reset data
        work_stamp_start = 0
        pause_stamp_start = 0
        task_record = {
            "events": [],
            "results": ""
        }
        rospy.loginfo("从pause状态到cancel状态")
    previous_status = status



    


if __name__ == "__main__":
    rospy.init_node("system_logger")
    sub = rospy.Subscriber("/galileo/status", GalileoStatus, galileo_status_logger, queue_size=10)
    log_pub = rospy.Publisher("/xiaoqiang_log", LogRecord, queue_size=10)
    while not rospy.is_shutdown():
        time.sleep(1)
