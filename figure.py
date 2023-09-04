# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

plt.close('all')

workdir = "/home/sh/code/LC_GINS/dataset/"

nav_path = workdir + "result.txt"

nav = np.loadtxt('/home/sh/code/LC_GINS/dataset/result.txt')
kfnav = np.loadtxt('/home/sh/opensrc/KF-GINS/dataset/KF_GINS_Navresult.nav')
gtnav = np.loadtxt('/home/sh/code/LC_GINS/dataset/truth.nav')

fignum = 0

fignum += 1
plt.figure(fignum)
plt.plot(nav[:, 1], nav[:, 2], 'r.-')
plt.plot(kfnav[:, 2], kfnav[:, 3], 'b.-')
plt.plot(gtnav[:, 2], gtnav[:, 3], 'g.-')
plt.xlabel('lat(deg)')
plt.ylabel('lon(deg)')
plt.title('horizontal position')
plt.legend(['common', 'KF-GINS', 'gt'])
plt.grid()
plt.axis('equal')
plt.show()


fignum += 1
plt.figure(fignum)
plt.plot(nav[:, 0], nav[:, 1], 'r.-')
plt.plot(kfnav[:, 1], kfnav[:, 2], 'b.-')
plt.plot(gtnav[:, 1], gtnav[:, 2], 'g.-')
plt.xlabel('time')
plt.ylabel('latitude(deg)')
plt.title('laitude')
plt.legend(['common', 'KF-GINS', 'gt'])
plt.grid()
plt.show()

fignum += 1
plt.figure(fignum)
plt.plot(nav[:, 0], nav[:, 2], 'r.-')
plt.plot(kfnav[:, 1], kfnav[:, 3], 'b.-')
plt.plot(gtnav[:, 1], gtnav[:, 3], 'g.-')
plt.xlabel('time')
plt.ylabel('longtitude(deg)')
plt.title('longtitude')
plt.legend(['common', 'KF-GINS', 'gt'])
plt.grid()
plt.show()

fignum += 1
plt.figure(fignum)
plt.plot(nav[:, 0], nav[:, 3], 'r.-')
plt.plot(kfnav[:, 1], kfnav[:, 4], 'b.-')
plt.plot(gtnav[:, 1], gtnav[:, 4], 'g.-')
plt.xlabel('time')
plt.ylabel('height(m)')
plt.title('height')
plt.legend(['common', 'KF-GINS', 'gt'])
plt.grid()
plt.show()

fignum += 1
plt.figure(fignum)
plt.plot(nav[:, 0], nav[:, 4], 'r.-')
plt.plot(kfnav[:, 1], kfnav[:, 5], 'b.-')
plt.plot(gtnav[:, 1], gtnav[:, 5], 'g.-')
plt.xlabel('time')
plt.ylabel('vel x (m/s)')
plt.title('vel x compare')
plt.legend(['common', 'KF-GINS', 'gt'])
plt.grid()
plt.show()

fignum += 1
plt.figure(fignum)
plt.plot(nav[:, 0], nav[:, 5], 'r.-')
plt.plot(kfnav[:, 1], kfnav[:, 6], 'b.-')
plt.plot(gtnav[:, 1], gtnav[:, 6], 'g.-')
plt.xlabel('time')
plt.ylabel('vel y (m/s)')
plt.title('vel y compare')
plt.legend(['common', 'KF-GINS', 'gt'])
plt.grid()
plt.show()


fignum += 1
plt.figure(fignum)
plt.plot(nav[:, 1], nav[:, 2], 'r.-')
plt.xlabel('lat(deg)')
plt.ylabel('lon(deg)')
plt.title('GIlib')
plt.grid()
plt.axis('equal')
plt.show()

fignum += 1
plt.figure(fignum)
plt.plot(nav[:, 0], nav[:, 4:7], '.-')
plt.xlabel('time')
plt.ylabel('vel')
plt.grid()
plt.title('GIlib')
plt.legend(['x', 'y', 'z'])
plt.show()

fignum += 1
plt.figure(fignum)
plt.plot(nav[:, 0], nav[:, 7:10], '.-')
plt.xlabel('time')
plt.ylabel('rpy')
plt.grid()
plt.title('GIlib')
plt.legend(['roll', 'pitch', 'yaw'])
plt.show()


fignum += 1
plt.figure(fignum)
plt.plot(kfnav[:, 2], kfnav[:, 3], 'r.-')
plt.xlabel('dr x')
plt.ylabel('dr y')
plt.title('kf-gins')
plt.grid()
plt.axis('equal')
plt.show()

fignum += 1
plt.figure(fignum)
plt.plot(kfnav[:, 1], kfnav[:, 5:8], '.-')
plt.xlabel('time')
plt.ylabel('vel')
plt.title('kf-gins')
plt.grid()
plt.legend(['x', 'y', 'z'])
plt.show()

fignum += 1
plt.figure(fignum)
plt.plot(kfnav[:, 1], kfnav[:, 9:12], '.-')
plt.xlabel('time')
plt.ylabel('att')
plt.title('kf-gins')
plt.grid()
plt.legend(['roll', 'pitch', 'yaw'])
plt.show()
