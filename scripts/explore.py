#!/usr/bin/env python3 
# -*- coding: utf-8 -*-

import rospy
import tf
import os
from marker_localization.msg import MarkerPoseArray
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray

class Robot:
  
  def __init__(self):
    
    self.listener = tf.TransformListener()
    self.tag_ids = [13, 14, 15]
    title = "Tag hunt began. Looking for tags %s"%(self.tag_ids)
    rospy.loginfo_once(title) 
    # Rate for publishing prints, specifically
    rospy.Rate(1)
    self.pursuit = None
    # Subscription to topic that has informations on all tags, their pose and their #id
    self.sub_detect_marker = rospy.Subscriber("/detected_markers/output", MarkerPoseArray, self.getTagInfo)
    # Subscription to topic that has information on move_base status
    self.sub_mb_status = rospy.Subscriber("/move_base/status", GoalStatusArray, self.mb_status_check)
    # Publish to move_base goal
    self.pub_mb_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    self.mb_status = -1
    self.explore_active = True
    self.find_tag = False
    self.case = 0
    self.goal_move_base(pos_x = 0, pos_y = 0)
    rospy.spin()

  # This method selects the very first tag to be approached, based on sight
  def getTagInfo(self, data):
    
    # Tries to find values that are given by the fiducial marker finder.
    try:
      self.pos_x = data.markers[0].marker_pose.translation.x
      self.pos_y = data.markers[0].marker_pose.translation.y
      self.pos_z = data.markers[0].marker_pose.translation.z
      self.ori_x = data.markers[0].marker_pose.rotation.x
      self.ori_y = data.markers[0].marker_pose.rotation.y
      self.ori_z = data.markers[0].marker_pose.rotation.z
      self.ori_w = data.markers[0].marker_pose.rotation.w
      self.tag_id = data.markers[0].marker_id
      
      # If it succeeds, the tag seen will be pursuited
      self.tagPursuit()
      
    except IndexError:
      rospy.loginfo("No tags in sight!")
  
  def mb_status_check(self, data):
    self.mb_status = data.status_list[-1].status if data.status_list else -1  
  
  def tagPursuit(self):
    
    print("Found tag id", self.tag_id)
    print("It's coords are:", self.pos_x, self.pos_y, self.pos_z)
    print("Actual status:", self.mb_status)
    
    # If a tag gets detected
    if not self.find_tag:
      # Tag detected flag ON
      self.find_tag = True
      # Explore-lite gets killed
      # if self.explore_active:
      #   os.system("rosnode kill /explore")
      #   self.explore_active = False
      #   self.pub_mb_goal.publish()
      
      # A case for ONE TIME: when the first tag is found
      if self.case == 0:
        # The transform from the tag found to the base_link. It will be fed to the goal_move_base
        trans, rot = self.listener.lookupTransform('/base_link', '/near'+'_'+str(self.tag_id), rospy.Time(0))
        print("Pursuited tag is at", trans)
        self.goal_move_base(pos_x = trans[0], pos_y = trans[1])
        # Case 0 won't happen EVER AGAIN!
        self.case = 1 
      # For every other self.case value:
      else:
        # Tag ID number will be used in place of the virtual TFs
        self.tag_id = self.tag_id + 1 if self.tag_id < 15 else 13
        print("Approaching tag", self.tag_id)
        
        trans, rot = self.listener.lookupTransform('/base_link', '/goal_tag'+'_'+str(self.tag_id), rospy.Time(0))
        self.goal_move_base(pos_x = trans[0], pos_y = trans[1])
    
    # Definir condição de aproximação
    if self.mb_status != 1:
      self.pub_mb_goal.publish()
      self.find_tag = False

  def goal_move_base(self, pos_x=0, pos_y=0):
    msg_move_to_goal = PoseStamped()
    msg_move_to_goal.pose.position.x = pos_x
    msg_move_to_goal.pose.position.y = pos_y
    msg_move_to_goal.pose.orientation.z = 0
    msg_move_to_goal.pose.orientation.w = 1
    msg_move_to_goal.header.frame_id = 'base_link'
    self.pub_mb_goal.publish(msg_move_to_goal)

if __name__ == "__main__":
  rospy.init_node("tag_pursuit", anonymous=True)
  Robot()

