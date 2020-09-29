#!/usr/bin/env python

"""
setup.py file to setup and compile Tracker python library
"""

from distutils.core import setup, Extension
import numpy

tracker_module = Extension('_Tracker', sources=['tracker_wrap.cxx', 'tracker.cpp'], language="c++",
                           include_dirs=[numpy.get_include()], extra_compile_args=["-std=c++17", "-O3"])

setup (name='Tracker', version='1.0', author="Benjamin Young", description="Class object for tracking features in images for EECS 467",
        ext_modules=[tracker_module], py_modules=["Tracker"])
