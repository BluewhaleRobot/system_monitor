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
from galileo_serial_server.msg import GalileoStatus
import logging
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
        self.secret = IOT_SECRET
        self.skip_count = 30
        self.lk = linkkit.LinkKit(
            host_name="cn-shanghai",
            product_key=IOT_KEY,
            device_name=self.id_to_device_name(),
            device_secret=self.secret)
        # self.lk.enable_logger(logging.DEBUG)
        

        self.connect_flag = False

        def on_connect(session_flag, rc, userdata):
            print("Connected")
            self.connect_flag = True

        def on_disconnect(rc, userdata):
            print("Disconnected")

        self.lk.on_connect = on_connect
        self.lk.on_disconnect = on_disconnect

        self.lk.connect_async()

        while not self.connect_flag:
            time.sleep(1)

        if is_robot:
            _rc, _mid = self.lk.subscribe_topic(
                self.lk.to_full_topic("user/galileo/cmds"))
            self.lk.subscribe_rrpc_topic("/user/check_permission")
        else:
            _rc, _mid = self.lk.subscribe_topic(
                self.lk.to_full_topic("user/galileo/status"))

        def on_topic_message(topic, payload, qos, userdata):
            if topic == self.lk.to_full_topic("user/galileo/cmds") and self.on_galileo_cmds is not None:
                self.on_galileo_cmds(payload)
            if topic == self.lk.to_full_topic("user/galileo/status") and self.on_status_update is not None:
                self.on_status_update(payload)
        
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

    def set_on_status_update(self, cb):
        self.on_status_update = cb

    def status_received(self, status):
        if self.skip_count >= 0:
            self.skip_count -= 1
            return
        self.skip_count = 5
        print("publish status")
        self.publish_status(galileo_status_to_json(status))


if __name__ == "__main__":
    client = IotClient(get_my_id())
    def on_cmd_received(cmd):
        print("command received")
        pass
    client.set_on_galileo_cmds(on_cmd_received)
    rospy.init_node("iot_client")
    
    rospy.Subscriber("/galileo/status", GalileoStatus, client.status_received)
    while not rospy.is_shutdown():
        time.sleep(1)
