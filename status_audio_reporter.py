#!/usr/bin/env python
# encoding=utf-8

import rospy
from std_msgs.msg import String
from galileo_serial_server.msg import GalileoStatus
import time
import rosservice
import subprocess
import os

PREVISOUS_STATUS = None
BLOCK_TIME_COUNT = 0
WORKING_TIME_COUNT = 0
STOP_TIME_COUNT = 0
PREVIOUS_GREETING_FLAG = False

if __name__ == "__main__":
    rospy.init_node("status_audio_reporter")
    audio_pub = rospy.Publisher("/xiaoqiang_tts/text", String, queue_size=10)

    def status_update_cb(status):
        global PREVISOUS_STATUS, PREVIOUS_GREETING_FLAG, BLOCK_TIME_COUNT, STOP_TIME_COUNT, WORKING_TIME_COUNT
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
        # 被挡住提示
        if status.targetStatus == 1 and abs(status.currentSpeedX) < 0.01 and abs(status.currentSpeedTheta) < 0.01:
            # 被人挡住了,在 WORKING 状态但是没有动
            BLOCK_TIME_COUNT += (1000 / 30)
        else:
            BLOCK_TIME_COUNT = 0
        if BLOCK_TIME_COUNT >= 5000:  # 等待5秒
            BLOCK_TIME_COUNT = -10000  # 每15秒说一次
            audio_pub.publish("您好，请让一下。赤兔机器人努力工作中")

        # 巡航提示
        if status.targetStatus == 1 and (abs(status.currentSpeedX) > 0.01 or abs(status.currentSpeedTheta) > 0.01):
            # 被人挡住了,在 WORKING 状态但是没有动
            WORKING_TIME_COUNT += (1000 / 30)
        else:
            WORKING_TIME_COUNT = 0
        if WORKING_TIME_COUNT >= 4000:  # 等待2秒
            WORKING_TIME_COUNT = 0  # 每2秒说一次
            audio_pub.publish("巡 逻 中 ")

        # 5min不动则关闭雷达
        if abs(status.currentSpeedX) < 0.01 and abs(status.currentSpeedTheta) < 0.01:
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

    while not rospy.is_shutdown():
        time.sleep(1)
