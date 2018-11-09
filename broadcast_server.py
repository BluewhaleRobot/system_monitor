#!/usr/bin/env python
# encoding=utf-8
# The MIT License (MIT)
#
# Copyright (c) 2018 Bluewhale Robot
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# Author: Randoms
#

import json
from socket import AF_INET, SO_BROADCAST, SOCK_DGRAM, SOL_SOCKET, socket
from getmac import get_mac_address

import rospy

from utils.config import BROADCAST_PORT_V2
from utils.utils import get_my_id

if __name__ == "__main__":
    rospy.init_node("broadcast_server")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # 配置udp广播
        s = socket(AF_INET, SOCK_DGRAM)
        s.bind(('', 0))
        s.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)

        data = json.dumps({
            "id": get_my_id(),
            "port": 11311,
            "mac": get_mac_address(),
        }, indent=4)
        # 发送广播包
        try:
            s.sendto(data, ('<broadcast>', BROADCAST_PORT_V2))
        except Exception as e:
            print(e)
            continue
        rate.sleep()
