#!/usr/bin/env python3
import sys
# sys.path.append('/home/michael/Software/opencv/installation/OpenCV-3.4.4/python/cv2')
# sys.path.insert(0, "/usr/lib32")
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

import rospy
# import tf
# import tf2_ros
import numpy as np
# from pyqtgraph.Qt import QtGui, QtCore
# import pyqtgraph as pg
# from matplotlib import pyplot as plt
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from underwater_snakerobot_controller.msg import HardwareCommand
from std_msgs.msg import Float64MultiArray
sys.path.append('/home/michael/Software/opencv/installation/OpenCV-3.4.4/python/cv2')
# sys.path.append('/lib/x86_64-linux-gnu')
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import csv
import imutils
import math
import os
import shlex
import subprocess
import tempfile
#OpenCV
import cv2
from cv_bridge import CvBridge#, CvBridgeError
#Parrot Anafi
import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, UserTakeOff, Landing
from olympe.messages.ardrone3.Piloting import moveBy
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged, AttitudeChanged, SpeedChanged
from olympe.messages.ardrone3.PilotingSettings import MaxAltitude, MaxTilt
from olympe.messages.ardrone3.SpeedSettings import MaxVerticalSpeed, MaxRotationSpeed
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
from olympe.messages import gimbal
from olympe.enums.ardrone3.PilotingState import FlyingStateChanged_State
#PID
from simple_pid import PID

import time

from squaternion import euler2quat, quat2euler, Quaternion
# sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

def nothing(x):
    pass

class AnafiExample:
    def __init__(self):
        # Create the olympe.Drone object from its IP address
        self.drone = olympe.Drone(
            "192.168.42.1",
            loglevel=0,
        )
        self.tempd = tempfile.mkdtemp(prefix="olympe_streaming_test_")
        print("Olympe streaming example output dir: {}".format(self.tempd))
        self.h264_frame_stats = []
        self.h264_stats_file = open(
            os.path.join(self.tempd, 'h264_stats.csv'), 'w+')
        self.h264_stats_writer = csv.DictWriter(
            self.h264_stats_file, ['fps', 'bitrate'])
        self.h264_stats_writer.writeheader()
        #Flag
        self.flag = 0
        self.state_display_flag = 0
        self.joyBtnFlag = 0
        self.joyBtnCheck = np.zeros(17)
        self.drone_state = "LANDED"
        self.commandState = "landed"
        self.flyingCMD = "none"
        #PID
        self.heightPos = 180.0
        self.heightCon = 0.0
        self.heightPID = PID(0.025, 0.0004, 0.0025, setpoint=0.0)
        self.heightPID.output_limits = (-87, 87)
        self.heightPID.sample_time = 0.1  # update every 0.1 seconds
        self.widthPID = PID(0.032, 0.0004, 0.0025, setpoint=0.0)
        self.widthPID.output_limits = (-87, 87)
        self.widthPID.sample_time = 0.1  # update every 0.1 seconds
        #Initialize joystick
        self.joy = Twist()
        rospy.Subscriber("joy", Joy, self.joy_cb)
        #Publish ROS image
        self.image_pub = rospy.Publisher("/camera/image_raw",Image, queue_size=1)
        self.imu_pub = rospy.Publisher("/imu/data",Imu, queue_size=1)
        self.bridge = CvBridge()
        self.gimbalCam = Image()
        self.yaw_absolute_offset = 0.0
        self.gimbalAttitude = Imu()
        self.frame_quat = Quaternion(1.0, 0.0, 0.0, 0.0)
        self.frame_quat_relative = Quaternion(1.0, 0.0, 0.0, 0.0)
        #Image processing
        self.drone_info = None
        self.proc_image = None

    def joyDriftCheck(self, input):
        zeroDrift = 0.25
        if input < zeroDrift and input > -zeroDrift:
            input = 0
        elif input > zeroDrift:
            input = (input-zeroDrift)/(1-zeroDrift)
        elif input < -zeroDrift:
            input = (input+zeroDrift)/(1-zeroDrift)
        return int(input*100)

    def joy_cb(self, data):
        #Hand launch
        if data.buttons[0] == 1 and self.joyBtnCheck[0] == 0:
            self.joyBtnFlag = self.joyBtnFlag +1;
            if self.joyBtnFlag >= 2:
                self.joyBtnFlag = self.joyBtnFlag % 2
            if self.joyBtnFlag == 0:
                self.commandState = "landed"
                self.drone(UserTakeOff(state=0, _no_expect=True)
                            & FlyingStateChanged(
                            state="landed", _timeout=0, _policy="check_wait")
                          ).wait()
                print("Landed Mode ON")
            elif self.joyBtnFlag == 1:
                self.commandState = "usertakeoff"
                self.drone(UserTakeOff(state=1, _no_expect=True)
                            & FlyingStateChanged(
                            state="usertakeoff", _timeout=0, _policy="check_wait")
                          ).wait()
                print("User Takeoff Mode ON")
            self.joyBtnCheck[0] = 1
        if data.buttons[0] == 0 and self.joyBtnCheck[0] == 1:
            self.joyBtnCheck[0] = 0
        #Take Off
        if data.buttons[12] == 1 and self.joyBtnCheck[12] == 0:
            self.commandState = "takeoff"
            self.drone(
                TakeOff(_no_expect=True)
                & FlyingStateChanged(
                    state="hovering", _timeout=10, _policy="check_wait")
            ).wait()
            print("Take off")
            self.joyBtnCheck[12] = 1
        if data.buttons[12] == 0 and self.joyBtnCheck[12] == 1:
            self.joyBtnCheck[12] = 0
        #Landing
        if data.buttons[14] == 1 and self.joyBtnCheck[14] == 0:
            self.commandState = "landing"
            self.drone(
                Landing()
                >> FlyingStateChanged(state="landed", _timeout=5)
            ).wait()
            print("Landing")
            self.joyBtnCheck[14] = 1
        if data.buttons[14] == 0 and self.joyBtnCheck[14] == 1:
            self.joyBtnCheck[14] = 0
        #Start Piloting
        if data.buttons[13] == 1 and self.joyBtnCheck[13] == 0:
            print("startpiloting")
            self.drone.start_piloting()
            self.joyBtnCheck[13] = 1
        if data.buttons[13] == 0 and self.joyBtnCheck[13] == 1:
            self.joyBtnCheck[13] = 0
        #Stop Piloting
        if data.buttons[15] == 1 and self.joyBtnCheck[15] == 0:
            print("stoppiloting")
            self.drone.stop_piloting()
            self.joyBtnCheck[15] = 1
        if data.buttons[15] == 0 and self.joyBtnCheck[15] == 1:
            self.joyBtnCheck[15] = 0
        #Flight Control
        self.joy.linear.x = self.joyDriftCheck(data.axes[3])
        self.joy.linear.y = -self.joyDriftCheck(data.axes[2])
        self.joy.linear.z = self.joyDriftCheck(data.axes[0])
        self.joy.angular.z= -self.joyDriftCheck(data.axes[1])

    def controller_cb(self, event):
        # print("self.joy.linear.x: ",self.joy.linear.x)
        # print("self.joy.linear.y: ",self.joy.linear.y)
        # print("self.joy.linear.z: ",self.joy.linear.z)
        # print("self.joy.angular.z: ",self.joy.angular.z)
        # if state_temp is "LANDED":
        #     self.drone_state == "LANDED"
        # elif state_temp is "USER_TAKEOFF":
        #     self.drone_state == "USER_TAKEOFF"
        # elif state_temp is "HOVERING":
        #     self.drone_state == "HOVERING"
        # elif state_temp is "FLYING":
        #     self.drone_state == "FLYING"
        # if ((self.drone_state == "hovering" or self.drone_state == "flying")
        #     and self.drone_state == "landing"):
        #     self.drone(
        #         Landing()
        #         >> FlyingStateChanged(state="landed", _timeout=5)
        #     ).wait()
        #Control part
        if self.drone_state == "HOVERING" or self.drone_state == "FLYING":
            self.drone.piloting_pcmd(self.joy.linear.y,
                                     self.joy.linear.x,
                                     self.joy.linear.z,
                                     self.joy.angular.z,0.05)
        #Image processing
        if self.flag == 2:
            # print(self.drone_state)
            if self.proc_image is not None:
        #     # Use OpenCV to convert the yuv frame to RGB
        #     ratio = cv2frame.shape[0] / float(resized.shape[0])
        #     cresized = cv2.medianBlur(resized,	5)
        #     cgray = cv2.cvtColor(cresized, cv2.COLOR_BGR2GRAY)
        #     # cresized = cv2.GaussianBlur(resized, (9, 9), 0)
        #
        #     hsv = cv2.cvtColor(cresized, cv2.COLOR_BGR2HSV)
        #     lower = np.array([70,5,5])
        #     upper = np.array([130,255,255])
        #     mask = cv2.inRange(hsv, lower, upper)
        #     output = cv2.bitwise_and(cresized, cresized, mask = mask)
        #     # convert the resized image to grayscale, blur it slightly,
        #     # and threshold it
        #     gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
        #     thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)[1]
        #     # find contours in the thresholded image
        #     cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
        #                             cv2.CHAIN_APPROX_SIMPLE)
        #     cnts = imutils.grab_contours(cnts)
        #     cv2.drawContours(resized, cnts, -1, (0, 255, 0), 2)
        #
        #     green_lower = np.array([40,45,5])
        #     green_upper = np.array([75,255,255])
        #     green_mask = cv2.inRange(hsv, green_lower, green_upper)
        #     green_output = cv2.bitwise_and(cresized, cresized, mask = green_mask)
        #     green_gray = cv2.cvtColor(green_output, cv2.COLOR_BGR2GRAY)
        #     # green_gray_blurred = cv2.medianBlur(green_gray,	5)
        #     circles	= cv2.HoughCircles(green_gray,cv2.HOUGH_GRADIENT,1,50,param1=130,param2=20,minRadius=20,maxRadius=90)
        #     if circles is not None:
        #         circles	= np.uint16(np.around(circles))
        #         circleNum = 0
        #         for	i in circles[0,:]:
        #             cv2.circle(resized,(i[0],i[1]),i[2],(0,255,0),6)#draw circle outline
        #             cv2.circle(resized,(i[0],i[1]),2,(0,0,255),3)#draw circle center
        #             circleNum = circleNum + 1
        #             self.heightPos = self.heightPos + i[1]
        #         self.heightCon = self.heightCon + self.heightPID(self.heightPos/circleNum-height/(2*ratio))
        #         self.heightPos = 0
        #     if heightCon > 88.0:
        #         heightCon = 88.0
        #     elif heightCon < -88.0:
        #         heightCon = -88.0
        #     self.gimbal_set_target = self.drone(gimbal.set_target(
        #         gimbal_id=0,
        #         control_mode="position",
        #         yaw_frame_of_reference="none",   # None instead of absolute
        #         yaw=0.0,
        #         pitch_frame_of_reference="absolute",
        #         pitch=self.heightCon,
        #         roll_frame_of_reference="none",     # None instead of absolute
        #         roll=0.0,
        #         )).wait()
                if self.drone_info[1] is not None:
                    # Write some Text
                    font                   = cv2.FONT_HERSHEY_SIMPLEX
                    fontScale              = 0.8
                    fontColor              = (255,255,0)
                    lineType               = 2
                    display = self.proc_image
                    if self.drone_info[1]["state"] is not None:
                        bottomLeftCornerOfText = (10,100)
                        cv2.putText(display,"Drone State: "+self.drone_info[1]["state"],
                                    bottomLeftCornerOfText,
                                    font, fontScale, fontColor, lineType)
                    if self.drone_info[1]["battery_percentage"] is not None:
                        bottomLeftCornerOfText = (10,130)
                        cv2.putText(display,"Battery: "+str(self.drone_info[1]["battery_percentage"])+"%",
                                    bottomLeftCornerOfText,
                                    font, fontScale, fontColor, lineType)
        #     # Use OpenCV to show this frame
        #     cv2.imshow("Olympe HSV Streaming", green_output)
        #     # cv2.imshow("Olympe Mask Streaming", thresh)
                    cv2.imshow("Olympe Original Streaming", display)
                    cv2.waitKey(1)  # please OpenCV for 1 ms...
            self.flag = 0
        self.flag = self.flag + 1

    def start(self):
        # Connect the the drone
        self.drone.connection()
        self.yaw_absolute_offset = self.drone.get_state(gimbal.attitude)['yaw_absolute']
        # self.drone(gimbal.relative_attitude_bounds(
        #     gimbal_id=0,
        #     min_pitch=-85.0,
        #     max_pitch=85.0,
        #     )).wait()
        self.drone(gimbal.set_max_speed(
            gimbal_id=0,
            yaw=30.0,
            pitch=30.0,
            roll=30.0,
            )).wait()
        self.gimbal_set_target = self.drone(gimbal.set_target(
            gimbal_id=0,
            control_mode="position",
            yaw_frame_of_reference="absolute",   # None instead of absolute
            yaw=0.0,
            pitch_frame_of_reference="absolute",
            pitch=0.0,
            roll_frame_of_reference="absolute",     # None instead of absolute
            roll=0.0,
            )).wait()

        # You can record the video stream from the drone if you plan to do some
        # post processing.
        # self.drone.set_streaming_output_files(
        #     h264_data_file=os.path.join(self.tempd, 'h264_data.264'),
        #     h264_meta_file=os.path.join(self.tempd, 'h264_metadata.json'),
        #     # Here, we don't record the (huge) raw YUV video stream
        #     # raw_data_file=os.path.join(self.tempd,'raw_data.bin'),
        #     # raw_meta_file=os.path.join(self.tempd,'raw_metadata.json'),
        # )

        # Setup your callback functions to do some live video processing
        self.drone.set_streaming_callbacks(
            raw_cb=self.yuv_frame_cb,
            h264_cb=self.h264_frame_cb
        )
        # Start video streaming
        self.drone.start_video_streaming()

    	#Start controller timer
        rospy.Timer(rospy.Duration(0.05), self.controller_cb)
        rospy.sleep(1.0)

    def stop(self):
        self.drone(UserTakeOff(state=0, _no_expect=True)).wait()
        # Properly stop the video stream and disconnect
        self.drone.stop_video_streaming()
        self.drone.disconnection()
        self.h264_stats_file.close()

    def yuv_frame_cb(self, yuv_frame):
        """
        This function will be called by Olympe for each decoded YUV frame.

            :type yuv_frame: olympe.VideoFrame
        """
        # the VideoFrame.info() dictionary contains some useful informations
        # such as the video resolution
        info = yuv_frame.info()

        if info["has_errors"] is not None:
            self.drone_info = yuv_frame.vmeta()
            height, width = info["yuv"]["height"], info["yuv"]["width"]

            # convert pdraw YUV flag to OpenCV YUV flag
            cv2_cvt_color_flag = {
                olympe.PDRAW_YUV_FORMAT_I420: cv2.COLOR_YUV2BGR_I420,
                olympe.PDRAW_YUV_FORMAT_NV12: cv2.COLOR_YUV2BGR_NV12,
            }[info["yuv"]["format"]]

            cv2frame = cv2.cvtColor(yuv_frame.as_ndarray(), cv2_cvt_color_flag)
            resized = imutils.resize(cv2frame, width=650)
            self.proc_image = resized

            #Publish Image and IMU Topic
            if self.drone_info[1] is not None:
                self.drone_state = self.drone_info[1]["state"]
                self.frame_quat = Quaternion(self.drone_info[1]["frame_quat"]["w"],
                                             self.drone_info[1]["frame_quat"]["x"],
                                             self.drone_info[1]["frame_quat"]["y"],
                                             self.drone_info[1]["frame_quat"]["z"])
            self.gimbalAttitude.orientation.x = self.frame_quat[1]
            self.gimbalAttitude.orientation.y = self.frame_quat[2]
            self.gimbalAttitude.orientation.z = self.frame_quat[3]
            self.gimbalAttitude.orientation.w = self.frame_quat[0]
            self.gimbalAttitude.angular_velocity.x = math.radians(self.drone.get_state(gimbal.attitude)['roll_relative'])
            self.gimbalAttitude.angular_velocity.y = math.radians(self.drone.get_state(gimbal.attitude)['pitch_relative'])
            self.gimbalAttitude.angular_velocity.z = math.radians(self.drone.get_state(gimbal.attitude)['yaw_relative'])
            self.gimbalAttitude.header.frame_id = "imu"
            timeNow = rospy.Time.now()
            self.gimbalAttitude.header.stamp = timeNow
            self.imu_pub.publish(self.gimbalAttitude)

            self.gimbalCam = self.bridge.cv2_to_imgmsg(resized, "bgr8")
            self.gimbalCam.header.frame_id = "camera_link"
            self.gimbalCam.header.stamp = timeNow
            self.image_pub.publish(self.gimbalCam)
            # yuv_frame.as_ndarray() is a 2D numpy array with the proper "shape"
            # i.e (3 * height / 2, width) because it's a YUV I420 or NV12 frame

    def h264_frame_cb(self, h264_frame):
        """
        This function will be called by Olympe for each new h264 frame.

            :type yuv_frame: olympe.VideoFrame
        """

        # Get a ctypes pointer and size for this h264 frame
        # frame_pointer, frame_size = h264_frame.as_ctypes_pointer()

        # For this example we will just compute some basic video stream stats
        # (bitrate and FPS) but we could choose to resend it over an another
        # interface or to decode it with our preferred hardware decoder..

        # Compute some stats and dump them in a csv file
        # info = h264_frame.info()
        # frame_ts = info["ntp_raw_timestamp"]
        # if not bool(info["h264"]["is_sync"]):
        #     if len(self.h264_frame_stats) > 0:
        #         while True:
        #             start_ts, _ = self.h264_frame_stats[0]
        #             if (start_ts + 1e6) < frame_ts:
        #                 self.h264_frame_stats.pop(0)
        #             else:
        #                 break
        #     self.h264_frame_stats.append((frame_ts, frame_size))
        #     h264_fps = len(self.h264_frame_stats)
        #     h264_bitrate = (
        #         8 * sum(map(lambda t: t[1], self.h264_frame_stats)))
        #     self.h264_stats_writer.writerow(
        #         {'fps': h264_fps, 'bitrate': h264_bitrate})

    def fly(self):
        # Takeoff, fly, land, ...
        print("Takeoff if necessary...")
        self.drone(
            FlyingStateChanged(state="hovering", _policy="check")
            | FlyingStateChanged(state="flying", _policy="check")
            | (
                GPSFixStateChanged(fixed=1, _timeout=10, _policy="check_wait")
                >> (
                    TakeOff(_no_expect=True)
                    & FlyingStateChanged(
                        state="hovering", _timeout=10, _policy="check_wait")
                )
            )
        ).wait()
        self.drone(MaxTilt(10)).wait().success()
        for i in range(3):
            print("Moving by ({}/3)...".format(i + 1))
            self.drone(moveBy(0, 0, 0, math.pi, _timeout=20)).wait().success()

        print("Landing...")
        self.drone(
            Landing()
            >> FlyingStateChanged(state="landed", _timeout=5)
        ).wait()
        print("Landed\n")

    def postprocessing(self):
        # Convert the raw .264 file into an .mp4 file
        h264_filepath = os.path.join(self.tempd, 'h264_data.264')
        mp4_filepath = os.path.join(self.tempd, 'h264_data.mp4')
        subprocess.run(
            shlex.split('ffmpeg -i {} -c:v copy {}'.format(
                h264_filepath, mp4_filepath)),
            check=True
        )

        # Replay this MP4 video file using the default video viewer (VLC?)
        # subprocess.run(
        #     shlex.split('xdg-open {}'.format(mp4_filepath)),
        #     check=True
        # )

if __name__ == '__main__':
    rospy.init_node('anafi')
    # rospy.Subscriber("/target_path", Odometry, update1, queue_size=1)
    streaming_example = AnafiExample()
    # Start the video stream
    streaming_example.start()
    # Perform some live video processing while the drone is flying
    # streaming_example.fly()
    # rospy.spin()

    try:
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
        # rospy.spin()
    except rospy.ROSInterruptException: pass
    # Stop the video stream
    streaming_example.stop()
    # Recorded video stream postprocessing
    # streaming_example.postprocessing()
    print("Shutdown!!!!!!\n")

# def euler_to_quaternion(self, roll, pitch, yaw):
#
#     qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
#     qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
#     qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
#     qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
#
#     return [qx, qy, qz, qw]
