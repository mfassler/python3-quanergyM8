
import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

from distutils.core import setup, Extension

module1 = Extension('cQuanergyM8',
                    sources = ['cpp/cQuanergyM8_module.cpp'])

setup(name='QuanergyM8',
      version='0.1',
      description='Parser for the Quanergy M8 LiDAR',
      packages=['quanergyM8'],
      ext_modules=[module1]
     )


