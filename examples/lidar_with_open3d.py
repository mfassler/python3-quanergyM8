#!/usr/bin/env python3

import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

import time
import socket
import struct
import numpy as np
import matplotlib.pyplot as plt
import open3d

from quanergyM8 import Quanergy_M8_Parser, MAGIC_SIGNATURE

if len(sys.argv) != 2:
    print('usage: %s ip_address' % (sys.argv[0]))
    sys.exit(1)

lidar_address = (sys.argv[1], 4141)


qparse = Quanergy_M8_Parser()

jet_colormap = plt.cm.jet(np.linspace(0,1,256))[:, :3]

vis = open3d.visualization.Visualizer()
vis.create_window(width=800, height=600) #, left=1100, right=50)
pcd = open3d.geometry.PointCloud()

_self = None

def pointcloud_callback(self):
    global vis
    global pcd
    global _self
    _self = self
    pointcloud = self.pointclouds[self.pc_idx]
    intensities = self.intensities[self.pc_idx]
    numpoints = self.num_points[self.pc_idx]

    print(" ******* NEW Pointcloud! %0.03f %d pts" % (time.time(), numpoints))

    # Convert the intensites into a Jet colormap:
    ii = intensities[:numpoints]
    colors = jet_colormap[ii]

    pcd.points = open3d.utility.Vector3dVector(pointcloud[:numpoints])
    pcd.colors = open3d.utility.Vector3dVector(colors)

    if self.number_of_pointclouds == 3:
        vis.add_geometry(pcd)

    _nothing_ = vis.update_geometry(pcd)
    _nothing_ = vis.poll_events()
    vis.update_renderer()


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


