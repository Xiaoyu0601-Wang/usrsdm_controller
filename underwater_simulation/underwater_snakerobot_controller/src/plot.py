#!/usr/bin/env python3

import rospy

import numpy as np
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
from matplotlib import pyplot as plt
from nav_msgs.msg import Odometry
from underwater_snakerobot_controller.msg import HardwareCommand
from std_msgs.msg import Float64MultiArray
import sys

import time

def update1(msg):
    global curve1, curve1_x, curve1_y

    curve1_x.append(msg.pose.pose.position.x)
    curve1_y.append(msg.pose.pose.position.y)
    # print("path: ", msg.pose.pose.position.x)
    curve1.setData(curve1_y, curve1_x)

def update2(msg):
    global curve2, curve2_x, curve2_y

    curve2_x.append(msg.pose.pose.position.x)
    curve2_y.append(msg.pose.pose.position.y)
    curve2.setData(curve2_y, curve2_x)

def update3(msg):
    global curve3, curve3_x, curve3_y

    curve3_x.append(msg.pose.pose.position.x)
    curve3_y.append(-msg.pose.pose.position.y)
    curve3.setData(curve3_y, curve3_x)

def update4(msg):
    global curve21, curve21_x, curve21_y, curve31, curve31_x, curve31_y, timeCount

    timeCount = timeCount + 0.2
    # dataCount = dataCount + 1
    # if dataCount > 100:
    #     pass
    curve21_x.append(timeCount)
    # curve21_y.append(msg.data[0])
    curve21_y.append(msg.jointPos1)
    curve21.setData(curve21_x, curve21_y)

    # dataCount1 = dataCount1 + 1
    curve31_x.append(timeCount)
    # curve21_y.append(msg.data[1])
    curve31_y.append(msg.screwVel1)
    curve31.setData(curve31_x, curve31_y)

    curve41_x.append(timeCount)
    # curve21_y.append(msg.data[1])
    curve41_y.append(msg.screwVel2)
    curve41.setData(curve41_x, curve41_y)

if __name__ == '__main__':
    app = QtGui.QApplication([])

    w = QtGui.QMainWindow()
    cw = pg.GraphicsLayoutWidget()
    # pg.mkPen(width=50)
    w.show()
    w.resize(700,1000)
    w.setCentralWidget(cw)
    w.setWindowTitle('Tracking Performance')

    ## variety of arrow shapes
    p = cw.addPlot(row=0, col=0, title="Tracking Performance")
    p.setLabel('left', "X", units='m')
    p.setLabel('bottom', "Y", units='m')
    curve1 = p.plot(pen={'color': (255,0,0), 'width': 5}, name="Red curve")
    curve2 = p.plot(pen={'color': (0,0,255), 'width': 5}, name="Blue curve")
    curve3 = p.plot(pen={'color': (0,255,0), 'width': 5}, name="Green curve")
    curve1_x = []
    curve1_y = []
    curve2_x = []
    curve2_y = []
    curve3_x = []
    curve3_y = []
    p.setRange(xRange=[-2,2])
    p.setRange(yRange=[-2,2])
    # p.plot(linewidth = 10)
    # p.enableAutoRange('xy', True)  ## stop auto-scal ing after the first data set is plotted

    timeCount = 0.0

    p2 = cw.addPlot(row=1, col=0, title="Joint Angle (Resolution)")
    # dataCount = 0
    curve21 = p2.plot(pen={'color': (255,0,0), 'width': 4}, name="Red curve")
    curve21_x = []
    curve21_y = []

    p3 = cw.addPlot(row=2, col=0, title="Screw Angular Velocity 1 (RPM)")
    # dataCount1 = 0
    curve31 = p3.plot(pen={'color': (255,0,0), 'width': 4}, name="Red curve")
    curve31_x = []
    curve31_y = []
    p3.enableAutoRange('xy', True)  ## stop auto-scal ing after the first data set is plotted

    p4 = cw.addPlot(row=3, col=0, title="Screw Angular Velocity 2 (RPM)")
    # dataCount1 = 0
    curve41 = p4.plot(pen={'color': (255,0,0), 'width': 4}, name="Red curve")
    curve41_x = []
    curve41_y = []
    p4.enableAutoRange('xy', True)
    ## Animated arrow following curve
    # c = p2.plot(x=np.sin(np.linspace(0, 2*np.pi, 1000)), y=np.cos(np.linspace(0, 6*np.pi, 1000)))
    # a = pg.CurveArrow(c)
    # a.setStyle(headLen=40)
    # p2.addItem(a)
    # anim = a.makeAnimation(loop=-1)
    # anim.start()

    rospy.init_node("plotter")
    rospy.Subscriber("/target_path", Odometry, update1, queue_size=1)
    rospy.Subscriber("/dataNavigator", Odometry, update2, queue_size=1)
    rospy.Subscriber("/RigidBody/odom", Odometry, update3, queue_size=1)
    rospy.Subscriber("/hardware_command", HardwareCommand, update4, queue_size=1)
    # rospy.Subscriber("/g500/thrusters_input", Float64MultiArray, update4, queue_size=1)

    try:
        while not rospy.is_shutdown():
            # update()
            QtGui.QApplication.instance().processEvents()
            rospy.sleep(0.2)
        if rospy.is_shutdown():
            # drone.disconnection()
            print("shutdown")

    except rospy.ROSInterruptException: pass
