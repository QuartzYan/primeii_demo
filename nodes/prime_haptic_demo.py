#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import time
import json
import socket 
import threading

import rospy 
from qr_sensor.msg import FingerSensor
from primeii_ros_msgs.msg import GloveHaptic

sub = None
pub = None
dongleid = None
handtype = None

fingerNames = [ "thumb", "index", "middle", "ring", "pinky" ]

def callBack(msg):
  haptic = GloveHaptic()
  haptic.header.frame_id = ''
  haptic.header.stamp = rospy.Time.now()
  haptic.dongleid = dongleid
  haptic.handtype = handtype
  sum = 0
  for i in range(2, len(msg.finger_sensors)):
    sum += msg.finger_sensors[i] 
  avg = (sum-3000)/8000.0
  if avg < 0:
    avg = 0
  elif avg > 1.0:
    avg = 1.0
  #rospy.loginfo(avg)
  for j in range(5):
    haptic.hapticPower[j] = avg
  pub.publish(haptic)

def main():
  global sub, pub, dongleid, handtype
  rospy.init_node("prime_haptic_demo")
  dongleid = rospy.get_param("~dongleid", default=3530882461)
  handtype = rospy.get_param("~handtype", default=2)
  #init Subscriber and Publisher
  sub = rospy.Subscriber("finger_sensors", FingerSensor, callBack)
  pub = rospy.Publisher("GloveHaptic", GloveHaptic, queue_size=1)
  
  rospy.spin()
  
if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass