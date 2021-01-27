#!/usr/bin/env python3 
# -*- coding: utf-8 -*-

import rospy
import tf
import os
import math
from marker_localization.msg import MarkerPoseArray
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray

class Robot:
  
  def __init__(self):
    
    self.mb_status = -1
    self.explore_active = True
    self.find_tag = False
    self.case = 0
    self.reset_time = True
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
    self.goal_move_base(pos_x = 0, pos_y = 0)
    rospy.spin()
    self.time = rospy.Time.now()
    

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
      self.tagsPursuit()
      
    except IndexError:
      rospy.loginfo("No tags in sight!")
  
  def mb_status_check(self, data):
    self.mb_status = data.status_list[-1].status if data.status_list else -1  
  
  def tagsPursuit(self):
    print("Found tag id", self.tag_id)
    print("Trans coords are:", "{:.3f}".format(self.pos_x), "{:.3f}".format(self.pos_y) , "{:.3f}".format(self.pos_z))
    print("Rot coords are:", "{:.3f}".format(self.ori_x), "{:.3f}".format(self.ori_y), "{:.3f}".format(self.ori_z), "{:.3f}".format(self.ori_w))
    print("Actual move_base status:", self.mb_status)
    print("")
    print("Rot coords in radians:")
    self.quaternion_to_euler(self.ori_w, self.ori_x, self.ori_y, self.ori_z )
    
    # If a tag gets detected
    if not self.find_tag:
      # Tag detected flag ON
      self.find_tag = True
      # Time counter. Every time this condition is met, self.time gets updated with 'now' time.
      # This condition is met after every goal_move_base method usage.
      if self.reset_time:
        self.time = rospy.Time.now()
        self.reset_time = False
      # Explore-lite gets killed
      # if self.explore_active:
      #   os.system("rosnode kill /explore")
      #   self.explore_active = False
      #   self.pub_mb_goal.publish()
      
      # A case for ONE TIME: when the first tag is found
      if self.case == 0 and ((rospy.Time.now() - self.time).to_sec() >= 5):
        # The transform from the tag found to the base_link. It will be fed to the goal_move_base
        trans, rot = self.listener.lookupTransform('/base_link', '/near'+'_'+str(self.tag_id), rospy.Time(0))
        # The transform from the camera frame to the tag frame. Used to correct the robot's orientation
        ctrans, crot = self.listener.lookupTransform('/camera_rgb_optical_frame', '/tag'+'_'+str(self.tag_id), rospy.Time(0))
        self.goal_move_base(pos_x = trans[0], pos_y = trans[1], ori_z=crot[2])
        # To correct the orientation, the move_base command must be related to the base_link, but the desired orientation comes from the tf relation between the camera_optical_frame and the tag.
        print("Pursuited tag is approximately", "{:.3f}".format(trans[0]), "meters ahead")
        # self.goal_move_base(ori_z=crot[2])
        
        # Case 0 won't happen EVER AGAIN!
        self.case = 1 
        self.reset_time = True
      # For every other self.case value:
      elif ((rospy.Time.now() - self.time).to_sec() >= 5):
        # Tag ID number will be used in place of the virtual TFs
        self.tag_id = self.tag_id + 1 if self.tag_id < 15 else 13
        print("Approaching tag", self.tag_id)
        trans, rot = self.listener.lookupTransform('/base_link', '/goal_tag'+'_'+str(self.tag_id), rospy.Time(0))
        self.goal_move_base(pos_x = trans[0], pos_y = trans[1])
        self.reset_time = True
      else:
        self.find_tag = False
    
    if self.mb_status != 1:
      self.find_tag = False

  def goal_move_base(self, pos_x=0, pos_y=0, ori_z=0, ori_w=0):
    msg_move_to_goal = PoseStamped()
    msg_move_to_goal.pose.position.x = pos_x
    msg_move_to_goal.pose.position.y = pos_y
    msg_move_to_goal.pose.orientation.z = ori_z
    msg_move_to_goal.pose.orientation.w = 1
    
    msg_move_to_goal.header.frame_id = 'base_link'
    self.pub_mb_goal.publish(msg_move_to_goal)
  
  def quaternion_to_euler(self, w, x, y, z):

    t0 = 2 * (w * x + y * z)
    t1 = 1 - 2 * (x * x + y * y)
    X = math.atan2(t0, t1)

    t2 = 2 * (w * y - z * x)
    t2 = 1 if t2 > 1 else t2
    t2 = -1 if t2 < -1 else t2
    Y = math.asin(t2)

    t3 = 2 * (w * z + x * y)
    t4 = 1 - 2 * (y * y + z * z)
    Z = math.atan2(t3, t4)

    print("{:.4f}".format(X), "{:.3f}".format(Y), "{:.3f}".format(Z))
    return X, Y, Z
  
  def euler_to_quaternion(self, phi, theta, psi):

    qw = math.cos(phi/2) * math.cos(theta/2) * math.cos(psi/2) + math.sin(phi/2) * math.sin(theta/2) * math.sin(psi/2)
    qx = math.sin(phi/2) * math.cos(theta/2) * math.cos(psi/2) - math.cos(phi/2) * math.sin(theta/2) * math.sin(psi/2)
    qy = math.cos(phi/2) * math.sin(theta/2) * math.cos(psi/2) + math.sin(phi/2) * math.cos(theta/2) * math.sin(psi/2)
    qz = math.cos(phi/2) * math.cos(theta/2) * math.sin(psi/2) - math.sin(phi/2) * math.sin(theta/2) * math.cos(psi/2)

    return [qw, qx, qy, qz]
  

if __name__ == "__main__":
  rospy.init_node("tag_pursuit", anonymous=True)
  Robot()

