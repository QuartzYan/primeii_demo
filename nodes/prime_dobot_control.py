#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import time
import json
import socket 
import threading

import rospy 
from dobot.srv import SetCmdTimeout, SetJOGCmd, SetEndEffectorGripper
from primeii_ros_msgs.msg import FingerFlex, GloveData, GlovesData

sub = None
cmdSrv = None
gripperSrv = None

fingerNames = [ "thumb", "index", "middle", "ring", "pinky" ]

def callBack(msg):
  #call gripper service
  if msg.glovesData[0].fingersFlex[0].Joint1Stretch > 0.6:
    if not cmdSrv(1, 1, False):
      rospy.logerr("Failed to call SetEndEffectorGripper")
  else:
    if not cmdSrv(1, 0, False):
      rospy.logerr("Failed to call SetEndEffectorGripper")
  #call SetJOGCmd service
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
  global sub, cmdSrv, gripperSrv
  rospy.init_node("prime_dobot_control")
  #init Subscriber and Publisher
  sub = rospy.Subscriber("GlovesData", GlovesData, callBack)
  #wait for this sevice to be running
  rospy.wait_for_service('/DobotServer/SetJOGCmd')
  rospy.wait_for_service('/DobotServer/SetCmdTimeout')
  rospy.wait_for_service('/DobotServer/SetEndEffectorGripper')
  #Create the connection to the service
  cmdSrv = rospy.ServiceProxy("/DobotServer/SetJOGCmd", SetJOGCmd)
  timeSrv = rospy.ServiceProxy("/DobotServer/SetCmdTimeout", SetCmdTimeout)
  gripperSrv = rospy.ServiceProxy("/DobotServer/SetEndEffectorGripper", SetEndEffectorGripper)
  #send the request through the connection
  if not timeSrv(3000):
    rospy.logerr("Failed to call SetCmdTimeout. Maybe DobotServer isn't started yet!")
  
  rospy.spin()
  
if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass