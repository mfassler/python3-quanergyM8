#!/usr/bin/env python3

import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

import time
import socket
import struct
import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv
import open3d

sys.path.append('/pubfiles/github/mfassler/python3-quanergyM8/build/lib.linux-x86_64-3.7')
from quanergyM8 import Quanergy_M8_Parser, MAGIC_SIGNATURE



REMOTE_HOST = "192.168.17.176"


REF_POINTCLOUD = np.load('../navigationByPointcloud/newRefPC.npy')
ref_pcd = open3d.geometry.PointCloud()
ref_pcd.points = open3d.utility.Vector3dVector(REF_POINTCLOUD)

colors = np.tile( [0, 0.8, 0], (len(REF_POINTCLOUD), 1))
ref_pcd.colors = open3d.utility.Vector3dVector(colors)

pcd = open3d.geometry.PointCloud()

qparse = Quanergy_M8_Parser()


udpsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udpsock.bind(("0.0.0.0", 0))



prev_Xfrm = np.identity(4)

def find_transform(src, target):
    global prev_Xfrm
    reg_p2p = open3d.registration.registration_icp(src, target, 80,
        prev_Xfrm, open3d.registration.TransformationEstimationPointToPoint())

    xfrm3d = np.copy(reg_p2p.transformation)
    prev_Xfrm = xfrm3d
    xfrm25d = np.delete(xfrm3d, 2, 0)
    xfrm2d = np.delete(xfrm25d, 2, 1)
    return xfrm2d



prev_position = 0
def forward_pointcloud_data(qparse):
    global prev_position
    if qparse.cur_numpoints > prev_position:
        someData = qparse.cur_pointcloud[prev_position:qparse.cur_numpoints, :2].astype(np.float16)
        #print(len(someData), 'points')
        udpPacket = struct.pack('I', qparse.number_of_pointclouds) + someData.tobytes()
        udpsock.sendto(udpPacket, (REMOTE_HOST, 11545))
    prev_position = qparse.cur_numpoints



def pointcloud_callback(self):
    global vis
    global pcd
    print(" ******* NEW Pointcloud!", time.time())

    pcd.points = open3d.utility.Vector3dVector(self.prev_pointcloud[:self.prev_numpoints])
    #pcd.colors = open3d.utility.Vector3dVector(colors)

    reXfrm = find_transform(ref_pcd, pcd)
    udpsock.sendto(reXfrm.tobytes(), (REMOTE_HOST, 11546))

qparse.pointcloud_callback = pointcloud_callback



#f = open('/data/Lidar_Capture/quanergy_capture-1.raw', 'rb')
#allData = f.read()
#f.close()
#print("expecting about ~%d packets" % (len(allData) / 6632))

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind(("0.0.0.0", 0))
sock.connect(("quanergy", 4141))



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
            forward_pointcloud_data(qparse)
            data = data[pos+size:]


