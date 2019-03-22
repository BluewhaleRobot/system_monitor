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
# Author: Randoms, Xiefusheng
#

from __future__ import absolute_import
import os
import re
import signal
import threading
import time
from .config import SHARPLINK_LOG_FILE

TIMEOUT = 5


def stop_process(target_process):
    if target_process.poll() is None:
        target_process.send_signal(signal.SIGINT)
        thread1 = threading.Thread(target=target_process.wait, args=())
        thread1.start()

    timecount = 0
    while timecount < TIMEOUT:
        timecount += 1
        time.sleep(1)
        try:
            os.killpg(target_process.pid, 0)
        except Exception:
            break

    if timecount >= TIMEOUT and target_process.poll() is None:
        os.killpg(target_process.pid, signal.SIGKILL)
        target_process.terminate()
        target_process.wait()


def get_my_id(myid=None):
    log_file = open(SHARPLINK_LOG_FILE)
    contents = log_file.read()
    log_file.close()
    mid_search = re.search(r"[0-9]+,\sID:\s(?P<id>[0-9A-F]{76})", contents)
    mid = mid_search.group("id")
    return mid
