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
        self.prev_pointcloud = np.empty((5400*3*8, 3), np.float32)
        self.prev_intensities = np.empty((5400*3*8), np.uint8)
        self.prev_numpoints = 0
        self.cur_pointcloud = np.empty((5400*3*8, 3), np.float32)
        self.cur_intensities = np.empty((5400*3*8), np.uint8)
        self.cur_numpoints = 0
        self.number_of_pointclouds = 0
        self.pointcloud_callback = _nothing_function_


    def parse(self, packet):
        signature, size, seconds, nanoseconds, version_major, version_minor, version_patch, \
            packet_type = struct.unpack('>IIIIBBBB', packet[:20])
        if packet_type == 0:
            self.parse_00(packet[20:])
        else:
            print("Unknown packet type: %d" % (packet_type))


    def parse_00(self, payload):
        assert len(payload) == 6612
        data = [None] * 50
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
            self.prev_pointcloud = self.cur_pointcloud
            self.prev_intensities = self.cur_intensities
            self.prev_numpoints = self.cur_numpoints
            self.cur_pointcloud = np.empty((5400*3*8, 3), np.float32)
            self.cur_intensities = np.empty((5400*3*8), np.uint8)
            self.cur_numpoints = 0
            self.pointcloud_callback(self)
        self._prev_position = position

        pts = _cq.parse_firing_data(f_data, self.cur_pointcloud, self.cur_intensities,
                                 self.cur_numpoints)
        self.cur_numpoints += pts


