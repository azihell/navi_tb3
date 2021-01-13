#!/usr/bin/env python3 
# -*- coding: utf-8 -*-

import rospy
import rostopic
import time
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal

class FindTags:
  
  def __init__(self):
    
    self.pose = PoseStamped()
    self.move_to_goal = PoseStamped()

    self.move_base_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
      
    # INIT VALUES for the non-emergency script
    
    # self.seq = None
    # self.zero_time = rospy.get_rostime().to_sec()
    # self.duration = rospy.Duration(nsecs=1000000000.000).to_sec()
    # self.previous = 0
    # self.previous2 = 0
    # print("Time now:", self.zero_time, "s. This time will be zeroed.")
    # print("Time update each:", self.duration, "s")

    tag0sub = rospy.Subscriber("/tag_0/stag_ros/markers", PoseStamped, callback=self.callback0, queue_size=10)
    # tag2sub = rospy.Subscriber("/tag_2/stag_ros/markers", PoseStamped, self.callback2, queue_size=10)

  def callback0(self, data):
    self.pose = data.pose
    if (self.pose.position.x != 0 and self.pose.position.y != 0 and self.pose.position.z != 0):
      rospy.loginfo_once("Tag 0 encontrada. Dirigindo-se a outro local")
      self.move_base(1, 1, 0, 1)
      tag1sub = rospy.Subscriber("/tag_1/stag_ros/markers", PoseStamped, callback=self.callback1, queue_size=10)

  def callback1(self, data):
    self.pose = data.pose
    if (self.pose.position.x != 0 and self.pose.position.y != 0 and self.pose.position.z != 0):
      rospy.loginfo_once("Tag 1 encontrada. Dirigindo-se a outro local")
      tag2sub = rospy.Subscriber("/tag_2/stag_ros/markers", PoseStamped, callback=self.callback2, queue_size=10)

  def callback2(self, data):
    self.pose = data.pose
    if (self.pose.position.x != 0 and self.pose.position.y != 0 and self.pose.position.z != 0):
      rospy.loginfo_once("Tag 2 encontrada. Dirigindo-se a outro local")
  
  def move_base(self, px, py, pz, pw):
    # rospy.loginfo_once("Move base started!")
    
    # client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    # # client.wait_for_server()
    
    # goal = MoveBaseGoal()
    # goal.target_pose.header.frame_id = "map"
    # goal.target_pose.header.stamp = rospy.Time.now()
    # goal.target_pose.pose.position.x = px
    # # goal.target_pose.pose.position.y = py
    # # goal.target_pose.pose.orientation.z = pz
    # goal.target_pose.pose.orientation.w = pw
    
    # client.send_goal(goal)
    # rospy.loginfo_once("Move base placeholder!")
    # wait = client.wait_for_result()
    # if not wait:
    #   rospy.logerr("Action server not available!")
    #   rospy.signal_shutdown("Action server not available! Shutting down")
    # else:
    #   return client.get_result
    while not rospy.is_shutdown():
      
      self.move_to_goal.pose.position.x = px
      self.move_to_goal.pose.position.y = py
      self.move_to_goal.pose.orientation.z = pz
      self.move_to_goal.pose.orientation.w = pw
      self.move_to_goal.header.frame_id = 'base_footprint'
    
      self.move_base_pub.publish(self.move_to_goal)
      rospy.loginfo_once("Move base job published")
  
  # Timed callback --- INCOMPLETE
    # This method is used to obtain seq values to perceive if the detection is lost (from a topic that can't be read for some reason).

  # def timecb(self): 
  #   rospy.loginfo_once("Timer activated")
  #   self.now = rospy.Time.now().to_sec()
  #   current = self.now - self.zero_time
  #   # store_seq = 0
    
  #   # Conditional runs it's body each self.duration seconds.
  #   if (current - self.previous >= self.duration):
  #     self.previous = current
  #     store_seq1 = self.seq
  #     print("Stored seq 1:", store_seq1, ". Timed at:", current)
      
  #   self.now = rospy.Time.now().to_sec()
  #   current2 = self.now - self.zero_time
  #   # store_seq2 = 0
    
  #   if (current2 - self.previous2 >= 1.1*self.duration):
  #     self.previous2 = current2
  #     store_seq2 = self.seq
  #     print("Stored seq 2:", store_seq2, ". Timed at:", current)
      
  #   # if (store_seq2 - store_seq) > 0:
  #   #   return Truemap
    
  # def callback0(self, data):
  #   self.seq = data.header.seq
  #   print(self.seq)

  #   # if self.timecb():
  #     # rospy.loginfo("Tags in sight")
  #   # if not self.timecb():
  #   #   rospy.loginfo("No tags in sight")
  
  # Variable snatcher --- INCOMPLETE
    # This method is supposed to save a variable whose value can be lost (from a topic that can't be read for some reason) to a file, to make it persistent
  
  # def seqSnatcher(self):
  #   snatch_seq = self.seq
  #   f = open('snatched_value.txt', 'w')
  #   f.write(snatch_seq)
    
  # EMERGENCY SCRIPT
    
  # def callback0(self, data):
  #   self.pose = data.pose    
  #   if (self.pose.position.x != 0 and self.pose.position.y != 0 and self.pose.position.z != 0):
  #     print("Tag #0 detected")
  #     self.tag0det = True
  #   # elif (selffinished.pose.position.x == 0 and self.pose.position.y == 0 and self.pose.position.z == 0):
  #   #   print("Tag #0 went MIA")
  #   #   self.tag0det = False

  # def callback1(self, data):
  #   self.pose = data.pose
  #   if (self.pose.position.x != 0 and self.pose.position.y != 0 and self.pose.position.z != 0):
  #     print("Tag #1 detected")
  #     self.tag1det = True
  #   # elif (self.pose.position.x == 0 and self.pose.position.y == 0 and self.pose.position.z == 0):
  #   #   print("Tag #1 went MIA")
  #   #   self.tag1det = False

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
      # seek.move_base_client(1, 0, 0, 1)
      # seek.callback1
      # seek.callback2
      rospy.spin()
    except rospy.ROSInterruptException:
      rospy.is_shutdown()
  
  # rospy.Timer(rospy.Duration(1.0/10.0), seek.callback0)
  # rospy.Timer(rospy.Duration(1.0/10.0), seek.callback1)