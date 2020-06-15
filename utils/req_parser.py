#!/usr/bin/env python3
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


class ReqParser(object):
    """docstring for ReqParser
       解析自定义数据包的类
    """

    def __init__(self, package_header=[205, 235, 215], buf_size=1024):
        super(ReqParser, self).__init__()
        self.data_cache = []
        self.package_header = package_header

    def unpack_req(self, req):
        res = []
        package_list = self.split_req(req)
        # process the first package
        complete_data = self.data_cache + package_list[0]
        package_list.remove(package_list[0])
        package_list = self.split_req(complete_data) + package_list

        for count in range(0, len(package_list)):
            if len(package_list[count]) != 0 and \
                    len(package_list[count]) == package_list[count][0] + 1:
                res.append(package_list[count][1:])
        last_one = package_list[-1:][0]  # the last one
        if len(last_one) == 0 or len(last_one) != last_one[0] + 1:
            self.data_cache = last_one
        return res

    def find_package_header(self, req):
        if len(req) < 3:
            return -1
        for count in range(0, len(req) - 2):
            if req[count] == self.package_header[0] and \
                    req[count + 1] == self.package_header[1] and \
                    req[count + 2] == self.package_header[2]:
                return count
        return -1

    def split_req(self, req):
        res = []
        start_index = 0
        new_index = 0
        while True:
            new_index = self.find_package_header(req[start_index:])
            if new_index == -1:
                break
            res.append(req[start_index: start_index + new_index])
            start_index = new_index + 3 + start_index
        res.append(req[start_index:])
        return res
