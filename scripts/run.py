#!/usr/bin/env python3 
# -*- coding: utf-8 -*-

import rospy
import rostopic
import time
from rostopic import ROSTopicHz
from rostopic import ROSTopicBandwidth
from geometry_msgs.msg import PoseStamped

class FindTags:
  
  def __init__(self):
    
    self.pose = PoseStamped()
    self.rate_print = rospy.Rate(1)
    self.seq = None
    self.zero_time = rospy.get_rostime().to_sec()
    self.duration = rospy.Duration(nsecs=500000000.000).to_sec()
    self.previous = 0
    self.previous2 = 0
    print("Time now:", self.zero_time, "s. This time will be zeroed.")
    print("Time update each:", self.duration, "s")

    tag0sub = rospy.Subscriber("/tag_0/stag_ros/markers", PoseStamped, callback=self.callback0, queue_size=10)
    # tag1sub = rospy.Subscriber("/tag_1/stag_ros/markers", PoseStamped, callback=self.callback1, queue_size=10)
    # tag2sub = rospy.Subscriber("/tag_2/stag_ros/markers", PoseStamped, self.callback2, queue_size=10)

  # Compares two "seq" values from the header of a marker message
  # def gett1(self):
  #   self.t1 = rospy.Time.now()
  #   return self.t1

  def timecb(self):
    rospy.loginfo_once("Timer activated")
    self.now = rospy.Time.now().to_sec()
    current = self.now - self.zero_time
    current2 = 1.05*(self.now -self.zero_time)
    store_seq = 0
    store_seq2 = 0
    # Conditional runs it's body each self.duration seconds.
    if (current - self.previous >= self.duration):
      # rospy.loginfo_once("Atual1", current)
      # rospy.loginfo_once("Anterior1:", self.previous)
      self.previous = current
      store_seq = self.seq
      # print("Stored time 1:", store_seq)
    
    if (current2 - self.previous2 >= 1.5*self.duration):
      # rospy.loginfo_once("Atual2", current2)
      # rospy.loginfo_once("Anterior2:", self.previous)
      self.previous2 = current2
      store_seq2 = self.seq
      # print("Stored time 2:", store_seq2)
    
    if (store_seq2 - store_seq) > 0:
      return True
    
  def callback0(self, data):
    self.seq = data.header.seq
    # self.timecb()
    if self.timecb():
      rospy.loginfo("Tags in sight")
    if not self.timecb():
      rospy.loginfo("No tags in sight")
    # self.pose = data.pose

    
    # if (self.pose.position.x != 0 and self.pose.position.y != 0 and self.pose.position.z != 0):
    #   print("Tag #0 detected")
    #   self.tag0det = True
    # if self.tag0det is not True:
    #   print("Nada")
    # elif (self.pose.position.x == 0 and self.pose.position.y == 0 and self.pose.position.z == 0):
    #   print("Tag #0 went MIA")
    #   self.tag0det = False

  # def callback1(self, data):
  #   self.pose = data.pose
  #   if (self.pose.position.x != 0 and self.pose.position.y != 0 and self.pose.position.z != 0):
  #     print("Tag #1 detected")
  #     self.tag0det = True
  #   elif (self.pose.position.x == 0 and self.pose.position.y == 0 and self.pose.position.z == 0):
  #     print("Tag #1 went MIA")
  #     self.tag0det = False

      
  # def callback1(self, data):
  #   self.pose = data.pose
  #   while self.pose:
  #     if (self.pose.position.x != 0 and self.pose.position.y != 0 and self.pose.position.z != 0):
  #       print("Tag #1 detected") queue_size=10)
    # tag2sub = rospy.Subscriber("/tag_2/stag_ros/markers", PoseStamped, self.callback2,
  #   while self.pose:
  #     if (self.pose.position.x != 0 and self.pose.position.y != 0 and self.pose.position.z != 0):
  #       print("Tag #2 detected")
  #       self.tag2det = True
  #     elif (self.pose.position.x == 0 and self.pose.position.y == 0 and self.pose.position.z == 0):
  #       print("Tag #1 went MIA")
  #       self.tag2det = False
  #     self.rate_print.sleep()


if __name__ == "__main__":
  
  rospy.init_node("turtle_navi", anonymous=True)
  seek = FindTags()
  while not rospy.is_shutdown():
    try:
      seek.callback0
      # seek.callback1
      # seek.callback2
      rospy.spin()
    except rospy.ROSInterruptException:
      rospy.is_shutdown()
  
  # rospy.Timer(rospy.Duration(1.0/10.0), seek.callback0)
  # rospy.Timer(rospy.Duration(1.0/10.0), seek.callback1)