#!/usr/bin/env python
#encoding=utf-8

import rospy
from std_msgs.msg import String
from galileo_serial_server.msg import GalileoStatus
from xqserial_server.srv import Shutdown, ShutdownRequest, ShutdownResponse
import time
import rosservice
import subprocess
import os
import time
import commands
from std_msgs.msg import Bool, UInt32, Int32
import threading
from utils.config import POWER_LOW

PREVISOUS_STATUS = None
BLOCK_TIME_COUNT = 0
STOP_TIME_COUNT = 0
PREVIOUS_GREETING_FLAG = False

POWER_NOW = 0.0
WARN_TIME_COUNT = 0
POWER_TIME_COUNT = 0

STATUS_LOCK = threading.Lock()
MOVE_FLAG =True

if __name__ == "__main__":
    rospy.init_node("status_audio_reporter")
    audio_pub = rospy.Publisher("/xiaoqiang_tts/text", String, queue_size=10)

    def moveflag_update_cb(status):
        global MOVE_FLAG
        with STATUS_LOCK:
            if status.data == 2:
                MOVE_FLAG = False
            else:
                MOVE_FLAG = True


    def status_update_cb(status):
        global PREVISOUS_STATUS, PREVIOUS_GREETING_FLAG, BLOCK_TIME_COUNT, STOP_TIME_COUNT, POWER_NOW, WARN_TIME_COUNT,POWER_TIME_COUNT
        global MOVE_FLAG
        with STATUS_LOCK:
            if PREVISOUS_STATUS == None:
                PREVISOUS_STATUS = status
                PREVIOUS_GREETING_FLAG = rospy.get_param(
                    "/xiaoqiang_greeting_node/is_enabled", False)
                return
            if PREVISOUS_STATUS.chargeStatus != 1 and status.chargeStatus == 1:
                audio_pub.publish("开始充电")
            if PREVISOUS_STATUS.chargeStatus != 2 and status.chargeStatus == 2:
                audio_pub.publish("充电完成")
            if PREVISOUS_STATUS.chargeStatus == 1 and status.chargeStatus == 0:
                audio_pub.publish("停止充电")
            if PREVISOUS_STATUS.mapStatus != 1 and status.mapStatus == 1:
                audio_pub.publish("开始创建地图")
            if PREVISOUS_STATUS.navStatus != 1 and status.navStatus == 1:
                audio_pub.publish("开始导航")
            if not PREVIOUS_GREETING_FLAG and rospy.get_param("/xiaoqiang_greeting_node/is_enabled", False):
                audio_pub.publish("开启迎宾模式")
            if PREVIOUS_GREETING_FLAG and not rospy.get_param("/xiaoqiang_greeting_node/is_enabled", False):
                audio_pub.publish("关闭迎宾模式")
            if status.navStatus == 1 and PREVISOUS_STATUS.targetNumID != 0 and status.targetNumID == 0:
                # 返回厨房提示
                #audio_pub.publish("好的，我回去了，您慢用！")
                pass
            # 被挡住提示
            if status.targetStatus == 1 and (not MOVE_FLAG or abs(status.currentSpeedX) < 0.01 and abs(status.currentSpeedTheta) < 0.01):
                # 被人挡住了,在 WORKING 状态但是没有动
                BLOCK_TIME_COUNT += (1000 / 30)
            else:
                BLOCK_TIME_COUNT = 0
            if BLOCK_TIME_COUNT >= 1000: # 等待3秒
                BLOCK_TIME_COUNT = -13000 # 每19秒说一次
                audio_pub.publish("请让开一下，谢谢，机器人努力工作中！")

            if status.power > 5.0:
                POWER_NOW = POWER_NOW*0.8 + status.power*0.2
                POWER_TIME_COUNT = POWER_TIME_COUNT +1

            if POWER_TIME_COUNT > 600 and POWER_NOW < POWER_LOW and status.power > 5.0 and status.targetStatus !=1 and status.chargeStatus == 0:
                POWER_TIME_COUNT = 601
                #需要提示
                WARN_TIME_COUNT += (1000 / 30)
                if WARN_TIME_COUNT >= 1000: # 等待1秒
                    WARN_TIME_COUNT = -26000 # 每27秒说一次
                    audio_pub.publish("电量低，请充满电后再继续使用！")

            # 被挡住提示
            if status.targetStatus == 1 and abs(status.currentSpeedX) < 0.01 and abs(status.currentSpeedTheta) < 0.01:
                # 被人挡住了,在 WORKING 状态但是没有动
                BLOCK_TIME_COUNT += (1000 / 30)
            else:
                BLOCK_TIME_COUNT = 0
            if BLOCK_TIME_COUNT >= 5000:
                BLOCK_TIME_COUNT = -10000
                audio_pub.publish("您好，请让一下。赤兔机器人努力工作中。")
            # 5min不动则关闭雷达
            if PREVISOUS_STATUS.navStatus==1 and status.targetStatus != 1 and abs(status.currentSpeedX) < 0.01 and abs(status.currentSpeedTheta) < 0.01:
                STOP_TIME_COUNT += (1000 / 30)
            else:
                STOP_TIME_COUNT = 0
            new_env = os.environ.copy()
            if STOP_TIME_COUNT >= 5 * 60 * 1000:
                if rospy.get_param("/rplidar_node_manager/keep_running", True):
                    rospy.set_param("/rplidar_node_manager/keep_running", False)
                    if rosservice.get_service_node("/stop_motor") is not None:
                        cmd = "rosservice call /stop_motor"
                        subprocess.Popen(
                            cmd, shell=True, env=new_env)
            else:
                if not rospy.get_param("/rplidar_node_manager/keep_running", True):
                    rospy.set_param("/rplidar_node_manager/keep_running", True)
                    if rosservice.get_service_node("/start_motor") is not None:
                        cmd = "rosservice call /start_motor"
                        subprocess.Popen(
                            cmd, shell=True, env=new_env)

            PREVIOUS_GREETING_FLAG = rospy.get_param(
                "/xiaoqiang_greeting_node/is_enabled", False)
            PREVISOUS_STATUS = status

    status_sub = rospy.Subscriber(
        "/galileo/status", GalileoStatus, status_update_cb)
    status_sub2 = rospy.Subscriber("/move_base/StatusFlag", Int32, moveflag_update_cb)
    while not rospy.is_shutdown():
        time.sleep(1)
