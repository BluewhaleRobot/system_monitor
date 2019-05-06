#!/usr/bin/env python
#encoding=utf-8

import rospy
from std_msgs.msg import String
from galileo_serial_server.msg import GalileoStatus
import time

PREVISOUS_STATUS = None
BLOCK_TIME_COUNT = 0

if __name__ == "__main__":
    rospy.init_node("status_audio_reporter")
    audio_pub = rospy.Publisher("/xiaoqiang_tts/text", String, queue_size=10)

    def status_update_cb(status):
        global PREVISOUS_STATUS, BLOCK_TIME_COUNT
        if PREVISOUS_STATUS == None:
            PREVISOUS_STATUS = status
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
        # 被挡住提示
        if status.targetStatus == 1 and abs(status.currentSpeedX) < 0.01 and abs(status.currentSpeedTheta) < 0.01:
            # 被人挡住了,在 WORKING 状态但是没有动
            BLOCK_TIME_COUNT += (1000 / 30)
        else:
            BLOCK_TIME_COUNT = 0
        if BLOCK_TIME_COUNT >= 5000: # 等待5秒
            BLOCK_TIME_COUNT = -10000 # 每15秒说一次
            audio_pub.publish("您好，请让一下。赤兔机器人努力工作中")

        PREVISOUS_STATUS = status

    status_sub = rospy.Subscriber("/galileo/status", GalileoStatus, status_update_cb)

    while not rospy.is_shutdown():
        time.sleep(1)
