#!/usr/bin/env python3

import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

import time
import socket
import select
import struct
import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv
import open3d


pcdsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
pcdsock.bind(("0.0.0.0", 11545))

navsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
navsock.bind(("0.0.0.0", 11546))



REF_POINTCLOUD = np.load('../navigationByPointcloud/newRefPC.npy')
ref_pcd = open3d.geometry.PointCloud()
ref_pcd.points = open3d.utility.Vector3dVector(REF_POINTCLOUD)

colors = np.tile( [0, 0.8, 0], (len(REF_POINTCLOUD), 1))
ref_pcd.colors = open3d.utility.Vector3dVector(colors)

jet_colormap = plt.cm.jet(np.linspace(0,1,256))[:, :3]

vis = open3d.visualization.Visualizer()
vis.create_window(width=800, height=600)

viewControl = vis.get_view_control()
param = open3d.io.read_pinhole_camera_parameters('../navigationByPointcloud/viewpoint.json')
pcd = open3d.geometry.PointCloud()
vis.add_geometry(ref_pcd)
vis.add_geometry(pcd)
viewControl.convert_from_pinhole_camera_parameters(param)








def make_map(width, height, grid_spacing):
    _map = np.ones((height, width, 3), np.uint8) * 255

    # Draw the grid:
    for x in np.arange(grid_spacing, width, grid_spacing):
        cv.line(_map, (x, 0), (x, height), (255,200,200), 2)
    for y in np.arange(grid_spacing, height, grid_spacing):
        cv.line(_map, (0, y), (width, y), (255,200,200), 2)

    return _map


avoidance_areas = make_map(400, 400, 100)
amap = np.copy(avoidance_areas)

cv.imshow('a map', amap)
#cv.moveWindow('a map', 560, 540)
cv.waitKey(1)


def render_nav(xfrm):
    heading = np.arctan2(xfrm[1,0], xfrm[1,1])
    amap = np.copy(avoidance_areas)

    startPos = np.dot(xfrm[:2, :2].T, xfrm[:2, 2])
    print(startPos)
    x0 = int(round(startPos[1] * 100)) + 200
    y0 = int(round(startPos[0] * 100)) + 200

    xyStart = (x0, y0)
    #print(xyStart)

    x1 = int(round(x0 + 100*np.sin(heading)))
    y1 = int(round(y0 - 100*np.cos(heading)))
    xyStop = (x1, y1)

    cv.arrowedLine(amap, xyStart, xyStop, (255,0,0), 2)
    cv.imshow('a map', amap)
    #cv.imwrite('images/2dnav-%05d.png' %(qparse.number_of_pointclouds), amap)
    cv.waitKey(1)



pointclouds = [
    np.empty((30000, 3), np.float64),
    np.empty((30000, 3), np.float64),
    np.empty((30000, 3), np.float64),
]

pc_max = [0, 0, 0]

pc_ring_idx = 0

frameNumber = 0
prev_pc_number = 0
def rx_pointcloud(pcdata):
    global prev_pc_number
    global pc_ring_idx
    global frameNumber
    pc_number, = struct.unpack('I', pcdata[:4])
    #print(pc_number)
    pc_data = np.frombuffer(pcdata[4:], np.float16).reshape((-1, 2))

    if pc_number != prev_pc_number:
        numPoints = pc_max[pc_ring_idx]
        pcd.points = open3d.utility.Vector3dVector(pointclouds[pc_ring_idx][:numPoints])
        colors = np.tile( [0, 0, 0.8], (numPoints, 1))
        pcd.colors = open3d.utility.Vector3dVector(colors)

        _nothing_ = vis.update_geometry()
        _nothing_ = vis.poll_events()
        vis.update_renderer()

        pc_ring_idx += 1
        if pc_ring_idx >= len(pointclouds):
            pc_ring_idx = 0
        pc_max[pc_ring_idx] = 0

    prev_pc_number = pc_number
    curPos = pc_max[pc_ring_idx]
    numNewPoints = pc_data.shape[0]
    newPos = curPos + numNewPoints
    pointclouds[pc_ring_idx][curPos:newPos, :2] = pc_data
    pc_max[pc_ring_idx] = newPos
    #print(pc_number)



t0 = time.time()
totalBytes = 0
while True:
    inputs, outputs, errors = select.select([pcdsock, navsock], [], [])
    for oneInput in inputs:
        if oneInput == pcdsock:
            #print('pcdsock')
            pcdPacket, addr = pcdsock.recvfrom(1500)
            rx_pointcloud(pcdPacket)
        elif oneInput == navsock:
            pkt, addr = navsock.recvfrom(256)
            if len(pkt) != 72:
                print("udp packet is wrong length")
            else:
                xfrm = np.frombuffer(pkt).reshape((3,3))
                render_nav(xfrm)
        else:
            print('wtf')



