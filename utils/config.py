#!/usr/bin/env python
# coding:utf-8
import numpy as np

HOST = ''  # should not be 127.0.0.1 or localhost
USERSOCKET_PORT = 20001  # 局域网udp命令监听端口
BROADCAST_PORT = 22001

MAX_VEL = 1.3
MAX_THETA = 3.0
POWER_LOW = 10.0

TF_ROT = np.array([[0., 0., 1.],
                   [-1., 0., 0.], [0., -1., 0.]])
TF_TRANS = np.array([0.33, 0.0, 0.])
