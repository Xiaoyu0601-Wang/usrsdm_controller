#!/usr/bin/env python3
import sys
import rospy
import numpy as np
#TCP communication with PC
from tcpcom import TCPServer
import time

import threading

HOST_IP_ADDRESS = "192.168.3.11"
IP_PORT = 5001

class usr_sdm:
    def __init__(self):
        self.server = TCPServer(IP_PORT, stateChanged = self.onStateChanged)
        self.isConnected = False
        self.tcpmsg = ""

    def onStateChanged(self, state, msg):
        if state == "LISTENING":
            print("Server:-- Listening...")
        elif state == "CONNECTED":
            self.isConnected = True
            print("Server:-- Connected to" + msg)
        elif state == "MESSAGE":
            print("Server:-- Message received:", msg)
            self.tcpmsg = msg
            # self.server.sendMessage("Button pressed")
        elif state == "TERMINATED":
            print("Server:-- TERMINATED")

    def start(self):
#         msg = can.Message(arbitration_id=0x001, data=[0x01, 0x02], extended_id=False)
#         self.can0.send(msg)
        print('msg sent')

        try:
            while not rospy.is_shutdown():
                msg = input()
                self.server.sendMessage(msg)
                # print(self.tcpmsg)
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass

    def stop(self):
        self.server.disconnect()
        self.server.terminate()
        print("Shutdown!!!!!!\n")

if __name__ == '__main__':
    rospy.init_node('usrsdm')
    USRSDM = usr_sdm()
    USRSDM.start()
    USRSDM.stop()
