#!/usr/bin/env python3

import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

import time
import socket
import struct
import numpy as np
import matplotlib.pyplot as plt
import cv2

from quanergyM8 import Quanergy_M8_Parser, MAGIC_SIGNATURE

if len(sys.argv) != 2:
    print('usage: %s ip_address' % (sys.argv[0]))
    sys.exit(1)

lidar_address = (sys.argv[1], 4141)


qparse = Quanergy_M8_Parser()

jet_colormap = plt.cm.jet(np.linspace(0,1,256))[:, :3]


_self = None

def pointcloud_callback(self):
    global ax
    global pcd
    global _self
    _self = self
    pointcloud = self.pointclouds[self.pc_idx]
    intensities = self.intensities[self.pc_idx]
    numpoints = self.num_points[self.pc_idx]
    print(" ******* NEW Pointcloud! %0.03f %d pts" % (time.time(), numpoints))

    # These are all the points from just the horizontal laser:
    horizIdxs = np.where(pointcloud[:numpoints, 2] == 0)
    horizGrid = pointcloud[horizIdxs]

    # Flip the Y axis:
    horizGrid[:,1] = -horizGrid[:,1]

    width_px = 800
    height_px = 600
    pixels_per_meter = 50

    img = np.zeros((height_px, width_px), np.uint8)

    # center the coords:
    horizGrid[:,0] += (width_px / 2) / pixels_per_meter
    horizGrid[:,1] += (height_px / 2) / pixels_per_meter

    # convert to pixel coordinates:
    coords = np.round(horizGrid * pixels_per_meter).astype(np.uint32)

    # remove out-of-bound coordinates:
    coords = coords[  coords[:, 0] >= 0]
    coords = coords[  coords[:, 1] >= 0]
    coords = coords[  coords[:, 0] < width_px]
    coords = coords[  coords[:, 1] < height_px]

    # Remember: Y coords become "row index" and X coords become "column index":
    img[ coords[:, 1], coords[:, 0] ] = 255

    cv2.imshow('asdf', img)
    cv2.waitKey(1)
    # Convert the intensites into a Jet colormap:
    #ii = intensities[:numpoints]
    #colors = jet_colormap[ii]



qparse.pointcloud_callback = pointcloud_callback


#f = open('/data/Lidar_Capture/quanergy_capture-1.raw', 'rb')
#allData = f.read()
#f.close()
#print("expecting about ~%d packets" % (len(allData) / 6632))

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind(("0.0.0.0", 0))
sock.connect(lidar_address)

while True:
    ch0 = ord(sock.recv(1))
    if ch0 == MAGIC_SIGNATURE[0]:
        ch1 = ord(sock.recv(1))
        if ch1 == MAGIC_SIGNATURE[1]:
            ch2 = ord(sock.recv(1))
            if ch2 == MAGIC_SIGNATURE[2]:
                ch3 = ord(sock.recv(1))
                if ch3 == MAGIC_SIGNATURE[3]:
                    header = sock.recv(16, socket.MSG_WAITALL)
                    size, seconds, nanoseconds, \
                    version_major, version_minor, version_patch, \
                    packet_type = struct.unpack('>IIIBBBB', header)

                    # size is either 6632 or 2224
                    # packet_type is either 0 or 4

                    if packet_type == 0 and size == 6632:
                        pkt = sock.recv(6612, socket.MSG_WAITALL)
                        qparse.parse_00(pkt)
                    else:
                        print('unsupported packet type: %d, %d bytes' % (packet_type, size))


