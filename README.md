# python3-quanergyM8
Python library to read live data from a Quanergy M8 LiDAR.

This library is stand-alone.  It does NOT require (or use) the Quanergy SDK.

![screenshot](/docs/quanergyLidar.jpg?raw=true)


## Hardware Requirements

* Quanergy M8 LiDAR

## Software Requirements

* Numpy
* The example viewer requires Open3d, but you can make your own viewer


## Getting started:

```bash
python3 setup.py install --user
pip3 install open3d --user

cd examples

python3 lidar_with_open3d.py ip_address_of_lidar
```
