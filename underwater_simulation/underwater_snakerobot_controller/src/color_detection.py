#!/usr/bin/env python3

import rospy

import numpy as np
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
from matplotlib import pyplot as plt
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import sys
sys.path.append('/home/michael/Software/opencv/installation/OpenCV-3.4.4/python/cv2')
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import csv
import imutils
import cv2
import math
import os
import shlex
import subprocess
import tempfile

import time

def nothing(x):
    pass

if __name__ == '__main__':
    rospy.init_node('color_detection')

    cap = cv2.VideoCapture(0)

    cv2.namedWindow("HSV Value")
    cv2.createTrackbar("H MIN", "HSV Value", 0, 179, nothing)
    cv2.createTrackbar("S MIN", "HSV Value", 0, 255, nothing)
    cv2.createTrackbar("V MIN", "HSV Value", 0, 255, nothing)
    cv2.createTrackbar("H MAX", "HSV Value", 179, 179, nothing)
    cv2.createTrackbar("S MAX", "HSV Value", 255, 255, nothing)
    cv2.createTrackbar("V MAX", "HSV Value", 255, 255, nothing)

    flag = 0

    while not rospy.is_shutdown():
        _, frame = cap.read()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        h_min = cv2.getTrackbarPos("H MIN", "HSV Value")
        s_min = cv2.getTrackbarPos("S MIN", "HSV Value")
        v_min = cv2.getTrackbarPos("V MIN", "HSV Value")
        h_max = cv2.getTrackbarPos("H MAX", "HSV Value")
        s_max = cv2.getTrackbarPos("S MAX", "HSV Value")
        v_max = cv2.getTrackbarPos("V MAX", "HSV Value")

        lower_blue = np.array([h_min, s_min, v_min])
        upper_blue = np.array([h_max, s_max, v_max])

        hsv_min="MIN H:{} S:{} V:{}".format(h_min,s_min,v_min)
        hsv_max = "MAX H:{} S:{} V:{}".format(h_max, s_max, v_max)

        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        result = cv2.bitwise_and(frame, frame, mask=mask)

        gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
        gray_blurred = cv2.medianBlur(gray,	5)
        circles	= cv2.HoughCircles(gray_blurred,cv2.HOUGH_GRADIENT,1,50,param1=100,param2=20,minRadius=20,maxRadius=130)
        if circles is not None:
            circles	= np.uint16(np.around(circles))
            for	i in circles[0,:]:
                #	draw	the	outer	circle
                cv2.circle(frame,(i[0],i[1]),i[2],(0,255,0),6)
                #	draw	the	center	of	the	circle
                cv2.circle(frame,(i[0],i[1]),2,(0,0,255),3)

        cv2.putText(frame, hsv_min, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(frame, hsv_max, (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.imshow("HSV Value", frame)
        cv2.imshow("Mask", mask)
        cv2.imshow("Frame Mask", result)

        key = cv2.waitKey(100)
        if key == 27:
            break

    rospy.spin()
    print("Shutdown!!!!!!\n")
