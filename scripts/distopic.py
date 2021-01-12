#!/usr/bin/env python3 
# -*- coding: utf-8 -*-

import rospy
import rostopic
from rostopic import ROSTopicHz
from rostopic import ROSTopicBandwidth
from geometry_msgs.msg import PoseStamped

def bla():
  print("Tô vivo")
  r = rostopic.ROSTopicHz(-1)
  tag0bwsub = rospy.Subscriber("/tag_0/stag_ros/markers", PoseStamped, r.callback_hz, callback_args="/tag_0/stag_ros/markers")
  rospy.sleep(1)
  r.print_hz(["/tag_0/stag_ros/markers"])
  rospy.spin()
  
try:
  rospy.init_node('dontlookatme')
  bla()
  rospy.spin()
except KeyboardInterrupt:
  print("Shutdown!")
  pass