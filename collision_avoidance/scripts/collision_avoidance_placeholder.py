#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import Float64

  
def euclidean_dist(x0, y0, x1, y1):
  return ((x1-x0)**2 + (y1-y0)**2)**0.5

def update_state_cb(msg, arg):
  global own_state
  if arg == "x":
    own_state[0] = msg.data
  elif arg == "y":
    own_state[1] = msg.data 
  else:  # if arg == "phi"
    phi = msg.data
    own_state[2] = 90 - phi / np.pi * 180 
    while own_state[2] >= 360:
      own_state[2] -= 360
    while own_state[2] < 0:
      own_state[2] += 360

def update_waypoint_cb(msg):
  global waypoint
  waypoint[0] = msg.linear.x
  waypoint[1] = msg.linear.y

def get_course(x0, y0, x1, y1):
  dx = x1 - x0
  dy = y1 - y0
  phi = np.arcsin(dy / euclidean_dist(x0, y0, x1, y1))
  if dx < 0:
    phi = np.pi - phi
  
  course = 90 - phi * 180 / np.pi
  while course >= 360:
    course -= 360
  while course < 0:
    course += 360
  return course


if __name__ == "__main__":
  rospy.init_node('collision_avoidance_placeholder')
  rate = rospy.Rate(10)
  
  own_state = [0, 0, 0]  # x, y, course in global frame
  
  own_x_sub = rospy.Subscriber('own_x', Float64, update_state_cb, callback_args="x")
  own_y_sub = rospy.Subscriber('own_y', Float64, update_state_cb, callback_args="y")
  own_course_sub = rospy.Subscriber('own_phi', Float64, update_state_cb, callback_args="phi")
  
  waypoint = [None, None]
  next_waypoint_sub = rospy.Subscriber('waypoint', Twist, update_waypoint_cb)
  
  local_target_course_pub = rospy.Publisher("local_target_course", Float64, queue_size=1)
  global_target_course_pub = rospy.Publisher("global_target_course", Float64, queue_size=1)
  local_waypoint_pub = rospy.Publisher("local_waypoint", Point, queue_size=1)
  global_waypoint_pub = rospy.Publisher("global_waypoint", Point, queue_size=1)
  
  while True:
    if not own_state:
      rospy.logerr("Failed to find the own state! The collision avoidance module has shut down.")
      break
    
    if waypoint[0] != None:
      x0 = own_state[0]
      y0 = own_state[1]
      x1 = waypoint[0]
      y1 = waypoint[1]
      
      course_0 = own_state[2]  # in degrees
      course_1 = get_course(x0, y0, x1, y1)  # in degrees
      # rospy.logerr("own_course " + str(course_0))
      # rospy.logerr("target_course " + str(course_1))
      
      d_course = course_1 - course_0
      while d_course > 180:
        d_course -= 180
      while d_course <= -180:
        d_course += 180
      
      # publish the target course
      local_target_course_pub.publish(d_course * np.pi / 180.0)
      global_target_course_pub.publish((course_1 - 90) * np.pi / 180.0)
      
      # publish the next waypoint in global frame
      global_waypoint = Point()
      global_waypoint.x, global_waypoint.y = x1, -y1
      global_waypoint_pub.publish(global_waypoint)
      
      # publish the next waypoint in local frame
      local_waypoint = Point()
      dist = euclidean_dist(x0, y0, x1, y1)
      local_waypoint.x, local_waypoint.y = dist*np.cos(d_course * np.pi / 180.0), dist*np.sin(d_course * np.pi / 180.0)
      local_waypoint_pub.publish(local_waypoint)
      
    rate.sleep()
    
    

