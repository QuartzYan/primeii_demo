#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import time
import json
import socket 
import threading

import rospy 
from dobot.srv import SetCmdTimeout, SetJOGCmd
from primeii_ros_msgs.msg import FingerFlex, GloveData, GlovesData

sub = None
cmdSrv = None

fingerNames = [ "thumb", "index", "middle", "ring", "pinky" ]

def callBack(msg):
  srv = SetJOGCmd()
  srv.isJoint = False
  srv.cmd = 0

  if msg.glovesData[0].fingersFlex[1].Joint1Stretch < -0.3:
    srv.cmd = 1
  if msg.glovesData[0].fingersFlex[1].Joint1Stretch > 0.3:
    srv.cmd = 2
  if msg.glovesData[0].fingersFlex[2].Joint1Stretch < -0.3:
    srv.cmd = 3
  if msg.glovesData[0].fingersFlex[2].Joint1Stretch > 0.3:
    srv.cmd = 4
  if msg.glovesData[0].fingersFlex[3].Joint1Stretch < -0.3:
    srv.cmd = 5
  if msg.glovesData[0].fingersFlex[3].Joint1Stretch > 0.3:
    srv.cmd = 6

  if not cmdSrv(srv.isJoint, srv.cmd):
    rospy.logerr("Failed to call SetJOGCmd")

def main():
  global sub, cmdSrv
  rospy.init_node("prime_dobot_control")
  #init Subscriber and Publisher
  sub = rospy.Subscriber("GlovesData", GlovesData, callBack)
  #wait for this sevice to be running
  rospy.wait_for_service('/DobotServer/SetJOGCmd')
  rospy.wait_for_service('/DobotServer/SetCmdTimeout')
  #Create the connection to the service
  cmdSrv = rospy.ServiceProxy("/DobotServer/SetJOGCmd", SetJOGCmd)
  timeSrv = rospy.ServiceProxy("/DobotServer/SetCmdTimeout", SetCmdTimeout)
  #send the request through the connection
  if not timeSrv(3000):
    rospy.logerr("Failed to call SetCmdTimeout. Maybe DobotServer isn't started yet!")
  
  rospy.spin()
  
if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass