#!/usr/bin/env python3

import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")


sys.path.append('../build/lib.linux-x86_64-3.7')

import numpy as np
import cQuanergyM8 as q



pointcloud = np.zeros((5020, 3), np.float32)
intensities = np.zeros(5020, np.uint8)


buf = 'a' * 132

q.parse_firing_data(buf, pointcloud, intensities, 4)


