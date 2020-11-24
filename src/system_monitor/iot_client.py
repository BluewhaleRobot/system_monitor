#!/usr/bin/env python
# encoding=utf-8

import utils.linkkit as linkkit
# from linkkit import linkkit
import time
import requests
import json
import sys
from utils.config import IOT_KEY, IOT_SECRET, IOT_PASSWORD
from utils.utils import get_my_id
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from galileo_serial_server.msg import GalileoStatus, GalileoNativeCmds
import logging
import threading
logging.basicConfig()


def galileo_status_to_json(galileo_status):
    return json.dumps({
        "timestamp": galileo_status.header.stamp.to_nsec() / 1000 / 1000,
        "angleGoalStatus": galileo_status.angleGoalStatus,
        "busyStatus": galileo_status.busyStatus,
        "chargeStatus": galileo_status.chargeStatus,
        "controlSpeedTheta": galileo_status.controlSpeedTheta,
        "controlSpeedX": galileo_status.controlSpeedX,
        "currentAngle": galileo_status.currentAngle,
        "currentPosX": galileo_status.currentPosX,
        "currentPosY": galileo_status.currentPosY,
        "currentSpeedTheta": galileo_status.currentSpeedTheta,
        "currentSpeedX": galileo_status.currentSpeedX,
        "gbaStatus": galileo_status.gbaStatus,
        "gcStatus": galileo_status.gcStatus,
        "loopStatus": galileo_status.loopStatus,
        "mapStatus": galileo_status.mapStatus,
        "navStatus": galileo_status.navStatus,
        "power": galileo_status.power,
        "targetDistance": galileo_status.targetDistance,
        "targetNumID": galileo_status.targetNumID,
        "targetStatus": galileo_status.targetStatus,
        "visualStatus": galileo_status.visualStatus,
    }, indent=4)

class IotClient():

    def __init__(self, id, is_robot=True):
        self.id = id
        self.on_galileo_cmds = None
        self.on_status_update = None
        self.on_test = None
        self.on_audio = None
        self.on_speed = None
        self.on_request = None
        self.secret = IOT_SECRET
        self.fast_update_flag = False
        self.skip_count = 30
        self.last_cmd_timestamp = 0
        self.lk = linkkit.LinkKit(
            host_name="cn-shanghai",
            product_key=IOT_KEY,
            device_name=self.id_to_device_name(),
            device_secret=self.secret)
        # self.lk.enable_logger(logging.DEBUG)
        

        self.connect_flag = False

        def on_connect(session_flag, rc, userdata):
            if rc == 0:
                rospy.loginfo("IOT connected")
                self.connect_flag = True

        def on_disconnect(rc, userdata):
            rospy.loginfo("IOT disconnected")

        self.lk.on_connect = on_connect
        self.lk.on_disconnect = on_disconnect
        
        self.lk.connect_async()

        while not self.connect_flag and not rospy.is_shutdown():
            time.sleep(1)
        if rospy.is_shutdown():
            return

        if is_robot:
            _rc, _mid = self.lk.subscribe_topic(
                self.lk.to_full_topic("user/galileo/cmds"))
            self.lk.subscribe_topic(
                self.lk.to_full_topic("user/test"))
            self.lk.subscribe_topic(
                self.lk.to_full_topic("user/speed"))
            self.lk.subscribe_topic(
                self.lk.to_full_topic("user/audio"))
            self.lk.subscribe_topic(
                self.lk.to_full_topic("user/galileo_api_bridge/request"))
            self.lk.subscribe_rrpc_topic("/user/check_permission")
        else:
            _rc, _mid = self.lk.subscribe_topic(
                self.lk.to_full_topic("user/galileo/status"))

        def on_topic_message(topic, payload, qos, userdata):
            rospy.logwarn("on_topic_message: " + topic)
            if topic == self.lk.to_full_topic("user/galileo/cmds") and self.on_galileo_cmds is not None:
                self.last_cmd_timestamp = int(time.time() * 1000)
                threading.Thread(target=self.on_galileo_cmds, args=(payload,)).start()
            if topic == self.lk.to_full_topic("user/galileo/status") and self.on_status_update is not None:
                threading.Thread(target=self.on_status_update, args=(payload,)).start()
            if topic == self.lk.to_full_topic("user/test") and self.on_test is not None:
                self.last_cmd_timestamp = int(time.time() * 1000)
                threading.Thread(target=self.on_test, args=(payload,)).start()
            if topic == self.lk.to_full_topic("user/audio") and self.on_audio is not None:
                self.last_cmd_timestamp = int(time.time() * 1000)
                threading.Thread(target=self.on_audio, args=(payload,)).start()
            if topic == self.lk.to_full_topic("user/speed") and self.on_speed is not None:
                self.last_cmd_timestamp = int(time.time() * 1000)
                threading.Thread(target=self.on_speed, args=(payload,)).start()
            if topic == self.lk.to_full_topic("user/galileo_api_bridge/request") and self.on_request is not None:
                self.last_cmd_timestamp = int(time.time() * 1000)
                threading.Thread(target=self.on_request, args=(payload,)).start()

        
        def on_rrpc_msg(rrpc_id, topic, payload, qos, userdata):
            req = json.loads(payload.decode())
            self.lk.thing_answer_rrpc(rrpc_id, json.dumps({
                "status": IOT_PASSWORD == req["password"],
                "description": payload.decode(),
            }))

        self.lk.on_topic_message = on_topic_message
        self.lk.on_topic_rrpc_message = on_rrpc_msg

    def id_to_device_name(self):
        return self.id[:32]

    def set_on_galileo_cmds(self, cb):
        self.on_galileo_cmds = cb

    def publish_galileo_cmds(self, cmd):
        _rc, _mid = self.lk.publish_topic(
            self.lk.to_full_topic("user/galileo/cmds"), cmd)

    def publish_status(self, status):
        _rc, _mid = self.lk.publish_topic(
            self.lk.to_full_topic("user/galileo/status"), status, qos=0)

    def publish_response(self, response):
        _rc, _mid = self.lk.publish_topic(
            self.lk.to_full_topic("user/galileo_api_bridge/response"), response, qos=0)

    def set_on_status_update(self, cb):
        self.on_status_update = cb
    
    def set_on_request(self, cb):
        self.on_request = cb

    def status_received(self, status):
        if self.fast_update_flag and self.skip_count > 30:
            self.skip_count = 30
        if self.skip_count >= 0:
            self.skip_count -= 1
            return
        self.skip_count = 30 * 60 # 默认1min上传一次
        if self.fast_update_flag:
            self.skip_count = 30 # 1s 更新一次
        rospy.logwarn(" update iot status")
        try:
            self.publish_status(galileo_status_to_json(status))
        except Exception:
            rospy.logwarn("update status failed")
    
    def set_on_test(self, cb):
        self.on_test = cb

    def set_on_audio(self, cb):
        self.on_audio = cb

    def set_on_speed(self, cb):
        self.on_speed = cb

def check_network_connection():
    try:
        requests.get("https://baidu.com", timeout=1)
        return True
    except Exception:
        return False


if __name__ == "__main__":
    rospy.init_node("iot_client")
    # 等待联网
    time.sleep(20)
    while not check_network_connection():
        rospy.logwarn("wait network")
        time.sleep(5)
    client = IotClient(get_my_id())
    galileo_cmd_pub = rospy.Publisher("/galileo/cmds", GalileoNativeCmds, queue_size=1)
    test_pub = rospy.Publisher("/pub_test", String, queue_size=1)
    audio_pub = rospy.Publisher("/xiaoqiang_tts/text", String, queue_size=1)
    speed_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    def on_cmd_received(cmd):
        galileo_cmd = GalileoNativeCmds()
        galileo_cmd.header.stamp = rospy.Time.now()
        galileo_cmd.length = len(cmd)
        galileo_cmd.data = cmd
        galileo_cmd_pub.publish(galileo_cmd)

    def on_test_received(msg):
        test_pub.publish(msg)
    
    def on_audio_received(msg):
        audio_pub.publish(msg)
    
    def on_speed_received(msg):
        msg = json.loads(msg)
        speed_twist = Twist()
        speed_twist.linear.x = msg["vLinear"]
        speed_twist.angular.z = msg["vAngle"]
        speed_pub.publish(speed_twist)
    
    def on_request_received(msg):
        msg = json.loads(msg)
        if not set(["uuid", "method", "url", "body"]).issubset(msg):
            return
        
        try:
            msg["url"] = "http://127.0.0.1:3546" + msg["url"]
            if msg["method"].lower() == "get":
                res = requests.get(msg["url"])
            if msg["method"].lower() == "post":
                res = requests.post(msg["url"],  json=json.loads(msg["body"]))
            if msg["method"].lower() == "put":
                res = requests.put(msg["url"], json=json.loads(msg["body"]))
            if msg["method"].lower() == "delete":
                res = requests.delete(msg["url"])
            
            client.publish_response(json.dumps({
                "uuid": msg["uuid"],
                "status_code": res.status_code,
                "body": res.content.decode("utf-8")
            }, indent=4))
            return
        except Exception as e:
            client.publish_response(json.dumps({
                "uuid": msg["uuid"],
                "status_code": 500,
                "body": json.dumps({
                    "status": "error",
                    "description": "server error"
                })
            }))
            rospy.logerr(e)
        

    client.set_on_galileo_cmds(on_cmd_received)
    client.set_on_test(on_test_received)
    client.set_on_audio(on_audio_received)
    client.set_on_speed(on_speed_received)
    client.set_on_request(on_request_received)

    rospy.Subscriber("/galileo/status", GalileoStatus, client.status_received)
    while not rospy.is_shutdown():
        time.sleep(1)
        current_time = int(time.time() * 1000)
        if current_time - client.last_cmd_timestamp > 2 * 60 * 1000: # 2min内没有收到新的指令，进入慢速状态上传模式
            client.fast_update_flag = False
        else:
            client.fast_update_flag = True
