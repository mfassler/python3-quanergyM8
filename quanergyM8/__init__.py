#!/usr/bin/env python3

import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

import time
import struct
import numpy as np

import cQuanergyM8 as _cq


'''
Please refer to the "M8 Sensor User's Guide", 
   chapter 5, "Getting TCP Ethernet Packets"
'''


MAGIC_SIGNATURE = b'\x75\xbd\x7e\x97'


def _nothing_function_(_self_):
    pass


class Quanergy_M8_Parser:
    def __init__(self):
        self._prev_position = 0

        # The lasers fire at 53828 Hz.
        # If the Lidar is rotating at 10 Hz, then that's ~5383 firings per rotation.
        # With 8 lasers, and 3 returns per laser, that about 5400*3*8 points per rotation.
        #
        # BUT!:  When the lidar is first spinning up, the rotation is less than 10Hz, so
        # there's more points per rotation.  So we need some extra space in our buffers

        self.pointclouds = [
            np.empty((6500*3*8, 3), np.float32),
            np.empty((6500*3*8, 3), np.float32),
        ]
        self.intensities = [
            np.zeros((6500*3*8), np.uint8),
            np.zeros((6500*3*8), np.uint8),
        ]
        self.num_points = [0, 0]
        self.pc_idx = 0

        self.number_of_pointclouds = 0
        self.pointcloud_callback = _nothing_function_


    def parse_00(self, payload):
        assert len(payload) == 6612
        for i in range(50):
            iStart = i*132
            iStop = iStart + 132
            self.parse_firing_data(payload[iStart:iStop])
        seconds, nanoseconds, api_version, status = struct.unpack('>LLHH', payload[iStop:])
        assert api_version == 5


    def parse_firing_data(self, f_data):
        assert len(f_data) == 132
        position, = struct.unpack('>H', f_data[0:2])
        if position < self._prev_position:
            self.number_of_pointclouds += 1
            self.pointcloud_callback(self)

            self.pc_idx += 1
            if self.pc_idx > 1:
                self.pc_idx = 0
            self.num_points[self.pc_idx] = 0

        self._prev_position = position

        #print('yo', self.pc_idx, self.number_of_pointclouds, self.num_points)
        pts = _cq.parse_firing_data(f_data,
                                    self.pointclouds[self.pc_idx],
                                    self.intensities[self.pc_idx],
                                    self.num_points[self.pc_idx])
        self.num_points[self.pc_idx] += pts


