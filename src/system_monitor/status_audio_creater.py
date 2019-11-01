#!/usr/bin/env python
#encoding=utf-8

import rospy
from std_msgs.msg import String
from galileo_serial_server.msg import GalileoStatus
import time
import rosservice
import subprocess
import os

PREVISOUS_STATUS = None
BLOCK_TIME_COUNT = 0
STOP_TIME_COUNT = 0
PREVIOUS_GREETING_FLAG = False

if __name__ == "__main__":
    rospy.init_node("status_audio_creater")
    audio_pub = rospy.Publisher("/xiaoqiang_tts/text", String, queue_size=10)
    time.sleep(3)
    audio_pub.publish("系统自检中")
    audio_pub.publish("系统自检完成")
    audio_pub.publish("底盘驱动程序没有启动，请检查底盘连接,并重启机器人")
    audio_pub.publish("检查底盘驱动程序")
    audio_pub.publish("未发现一号底盘串口设备，请检查一号底盘串口连接,并重启机器人")
    audio_pub.publish("检查一号底盘串口设备")
    audio_pub.publish("未发现二号底盘串口设备，请检查二号底盘串口连接,并重启机器人")
    audio_pub.publish("检查二号底盘串口设备")
    audio_pub.publish("环境传感器驱动程序没有启动，请检查环境传感器连接,并重启机器人")
    audio_pub.publish("检查环境传感器驱动程序")
    audio_pub.publish("未发现环境传感器串口设备，请检查环境传感器连接,并重启机器人")
    audio_pub.publish("前部摄像头驱动程序未启动，请检查摄像头连接,并重启机器人")
    audio_pub.publish("后部摄像头驱动程序未启动，请检查摄像头连接,并重启机器人")
    audio_pub.publish("未发现前部摄像头设备，请检查摄像头连接,并重启机器人")
    audio_pub.publish("未发现后部摄像头设备，请检查摄像头连接,并重启机器人")
    audio_pub.publish("检查后部摄像头驱动程序")
    audio_pub.publish("检查前部摄像头驱动程序")
    audio_pub.publish("检查后部摄像头设备")
    audio_pub.publish("检查前部摄像头设备")

    audio_pub.publish("激光雷达 驱动未启动，请检查激光雷达 连接,并重启机器人")
    audio_pub.publish("检查激光雷达 设备")
    audio_pub.publish("检查激光雷达 驱动程序")
    audio_pub.publish("未发现激光雷达 设备，请检查激光雷达 连接,并重启机器人")

    audio_pub.publish("自动充电驱动未启动，请检查自动充电模块 连接,并重启机器人")
    audio_pub.publish("检查自动充电驱动程序")
    audio_pub.publish("未发现自动充电设备，请检查自动充电连接,并重启机器人")
    audio_pub.publish("检查自动充电设备")

    audio_pub.publish("获取电池电压失败，请检查底盘连接，并重启机器人")
    audio_pub.publish("检查电池电压")
    audio_pub.publish("电池电压过低，请及时充电")

    audio_pub.publish("开始充电")
    audio_pub.publish("充电完成")
    audio_pub.publish("停止充电")
    audio_pub.publish("开始创建地图")
    audio_pub.publish("开始导航")
    audio_pub.publish("开启迎宾模式")
    audio_pub.publish("关闭迎宾模式")
    audio_pub.publish("开始闭环优化，请等待一段时间")
    audio_pub.publish("闭环优化完成，请继续操作")
    audio_pub.publish("您好，请让一下。机器人 努力工作中")
    audio_pub.publish("巡 逻 中 ")
