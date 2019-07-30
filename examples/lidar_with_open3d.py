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

sys.path.append('/pubfiles/github/mfassler/python3-quanergyM8/build/lib.linux-x86_64-3.7')
from quanergyM8 import Quanergy_M8_Parser, MAGIC_SIGNATURE

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
    print(" ******* NEW Pointcloud!", time.time())
    # Convert the intensites into a Jet colormap:
    ii = self.prev_intensities[:self.prev_numpoints]
    scale = 255 / ii.max()
    ii2 = ii * scale
    ii3 = np.round(ii2).astype(np.uint8)
    colors = jet_colormap[ii3]

    pcd.points = open3d.utility.Vector3dVector(self.prev_pointcloud[:self.prev_numpoints])
    pcd.colors = open3d.utility.Vector3dVector(colors)

    if self.number_of_pointclouds == 3:
        vis.add_geometry(pcd)

    _nothing_ = vis.update_geometry()
    _nothing_ = vis.poll_events()
    vis.update_renderer()


qparse.pointcloud_callback = pointcloud_callback


#f = open('/data/Lidar_Capture/quanergy_capture-1.raw', 'rb')
#allData = f.read()
#f.close()
#print("expecting about ~%d packets" % (len(allData) / 6632))

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind(("0.0.0.0", 0))
sock.connect(("192.168.11.103", 4141))

data = b''
while True:
    data += sock.recv(6632)
    if len(data) > 6632:
        pos = data.find(MAGIC_SIGNATURE)
        size, = struct.unpack('>I', data[pos+4 : pos+8])
        assert size > 5500
        assert size < 7500
        if len(data) > pos+size:
            pkt = data[pos:pos+size]
            qparse.parse(pkt)
            data = data[pos+size:]


