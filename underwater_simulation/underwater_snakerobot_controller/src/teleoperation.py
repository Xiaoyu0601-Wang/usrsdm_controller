#!/usr/bin/env python
import sys
sys.path.append("../../underwater_sensor_msgs/msg")

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from math import fabs

from HardwareCommand.msg import HardwareCommand

global lastData


def joyChanged(data):
    lastData = None
    lastData = data
    # print(data)

if __name__ == '__main__':
    rospy.init_node('teleop', anonymous=True)

    r = 20
    rate = rospy.Rate(r)

    hardwareCommand=HardwareCommand()

    # msg = PoseStamped()
    # msg.header.seq = 0
    # msg.header.stamp = rospy.Time.now()
    # msg.header.frame_id = worldFrame
    # msg.pose.position.x = x
    # msg.pose.position.y = y
    # msg.pose.position.z = z
    # yaw = 0
    # quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
    # msg.pose.orientation.x = quaternion[0]
    # msg.pose.orientation.y = quaternion[1]
    # msg.pose.orientation.z = quaternion[2]
    # msg.pose.orientation.w = quaternion[3]

    pub = rospy.Publisher("/hardware_command", HardwareCommand, queue_size=1)
    rospy.Subscriber(joy_topic, Joy, joyChanged)

    while not rospy.is_shutdown():
        if lastData != None:
            if fabs(lastData.axes[1]) > 0.05:
                # msg.pose.position.z += lastData.axes[1] / r / 2
                hardwareCommand.screwVel1 = lastData.axes[1]*100
                hardwareCommand.screwVel2 = lastData.axes[1]*100
                print(lastData.axes[1])
            # if fabs(lastData.axes[3]) > 0.1:
            #     # msg.pose.position.x += lastData.axes[3] / r * 1
            if fabs(lastData.axes[2]) > 0.05:
                hardwareCommand.jointPos1 = lastData.axes[2]*500+2000
                # msg.pose.position.y += lastData.axes[2] / r * 1
            # if fabs(lastData.axes[0]) > 0.1:
            #     yaw = lastData.axes[0] / r * 2
        #     quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        #     msg.pose.orientation.x = quaternion[0]
        #     msg.pose.orientation.y = quaternion[1]
        #     msg.pose.orientation.z = quaternion[2]
        #     msg.pose.orientation.w = quaternion[3]
        #     # print(pose)
        # msg.header.seq += 1
        # msg.header.stamp = rospy.Time.now()
        pub.publish(hardwareCommand)
        rate.sleep()
