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


REF_POINTCLOUD = np.load('refPC.npy')
ref_pcd = open3d.geometry.PointCloud()
ref_pcd.points = open3d.utility.Vector3dVector(REF_POINTCLOUD)

colors = np.tile( [0, 0.8, 0], (len(REF_POINTCLOUD), 1))
ref_pcd.colors = open3d.utility.Vector3dVector(colors)

sys.path.append('/pubfiles/github/mfassler/python3-quanergyM8/build/lib.linux-x86_64-3.7')
from quanergyM8 import Quanergy_M8_Parser, MAGIC_SIGNATURE

qparse = Quanergy_M8_Parser()

jet_colormap = plt.cm.jet(np.linspace(0,1,256))[:, :3]

vis = open3d.visualization.Visualizer()
vis.create_window(width=800, height=600) #, left=1100, right=50)

viewControl = vis.get_view_control()
param = open3d.io.read_pinhole_camera_parameters('viewpoint.json')
pcd = open3d.geometry.PointCloud()




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



def make_map(width, height, grid_spacing):
    _map = np.ones((height, width, 3), np.uint8) * 255

    # Draw the grid:
    for x in np.arange(grid_spacing, width, grid_spacing):
        cv.line(_map, (x, 0), (x, height), (255,200,200), 2)
    for y in np.arange(grid_spacing, height, grid_spacing):
        cv.line(_map, (0, y), (width, y), (255,200,200), 2)

    # Radial gridlines:
    #max_radius = np.sqrt(height**2 + (width/2)**2)
    #x_center = int(round(width/2))
    #for r in np.arange(grid_spacing, max_radius, grid_spacing):
    #    cv.circle(_map, (x_center, height), int(r), (255,128,128), 2)

    return _map

avoidance_areas = make_map(400, 400, 100)
amap = np.copy(avoidance_areas)

cv.imshow('a map', amap)
#cv.moveWindow('a map', 560, 540)
cv.waitKey(1)






def render_vis(xfrm):
    heading = np.arctan2(xfrm[1,0], xfrm[1,1])
    amap = np.copy(avoidance_areas)

    startPos = np.dot(xfrm[:2, :2].T, xfrm[:2, 2])
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




def pointcloud_callback(self):
    global vis
    global pcd
    print(" ******* NEW Pointcloud!", time.time())

    # Convert the intensites into a Jet colormap:
    ii = self.prev_intensities[:self.prev_numpoints]
    scale = 255 / ii.max()
    ii2 = ii * scale
    ii3 = np.round(ii2).astype(np.uint8)
    colors = jet_colormap[ii3]

    pcd.points = open3d.utility.Vector3dVector(self.prev_pointcloud[:self.prev_numpoints])
    pcd.colors = open3d.utility.Vector3dVector(colors)

    reXfrm = find_transform(ref_pcd, pcd)
    #print(reXfrm)
    #heading = np.arctan2(reXfrm[1,0], reXfrm[0,0])

    if self.number_of_pointclouds == 3:
        vis.add_geometry(pcd)
        vis.add_geometry(ref_pcd)
        viewControl.convert_from_pinhole_camera_parameters(param)

    _nothing_ = vis.update_geometry(pcd)
    _nothing_ = vis.update_geometry(ref_pcd)
    _nothing_ = vis.poll_events()
    vis.update_renderer()
    #vis.capture_screen_image('images/pointcloud-%05d.png' %(qparse.number_of_pointclouds))
    render_vis(reXfrm)


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
            data = data[pos+size:]


