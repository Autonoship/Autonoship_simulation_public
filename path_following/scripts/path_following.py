#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

  
def euclidean_dist(x0, y0, x1, y1):
  return ((x1-x0)**2 + (y1-y0)**2)**0.5

def update_state_cb(msg, arg):  # in gazebo frame
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

def update_goal_cb(msg, goal):
  goal[0] = int(msg.linear.x) / 10
  goal[1] = int((35020 - msg.linear.y) / 10)

def update_plan_cb(msg):  # use gazebo frame
  global plan
  global new_plan
  new_plan = True
  plan = list(msg.data)


if __name__ == "__main__":
  rospy.init_node('path_following')
  rate = rospy.Rate(10)
  
  new_plan = False
  plan = []
  plan_sub = rospy.Subscriber("global_plan", Float64MultiArray, update_plan_cb)
  
  next_waypoint_pub = rospy.Publisher("waypoint", Twist, queue_size=1)
  
  dist_threshold = 1000  # meters
  own_state = [0, 0, 0]  # x, y, course in global frame
  
  own_x_sub = rospy.Subscriber('own_x', Float64, update_state_cb, callback_args="x")
  own_y_sub = rospy.Subscriber('own_y', Float64, update_state_cb, callback_args="y")
  own_course_sub = rospy.Subscriber('own_phi', Float64, update_state_cb, callback_args="phi")
  
  while True:
    if not own_state:
      rospy.logerr("Failed to find the own state! The path following module has shut down.")
      break
    
    
    # placeholder. the format of own_state is TBD
    x0 = own_state[0]
    y0 = own_state[1]
    while plan:
      if new_plan:  # when get a new plan, pop out the starting point, and take the second as the candidate waypoint
        plan.pop(0)
        plan.pop(0)
        new_plan = False
        
      x1 = plan[0]
      y1 = plan[1]
      if euclidean_dist(x0, y0, x1, y1) > dist_threshold:
        # rospy.logerr(str(x0) + " " + str(y0) + " " + str(x1) + " " + str(y1))
        waypoint = Twist()
        waypoint.linear.x = x1
        waypoint.linear.y = y1
        next_waypoint_pub.publish(waypoint)
        break
      else:  # Have reached the first waypoint. Pop the first one.
        plan.pop(0)
        plan.pop(0)
    
    rate.sleep()
    
    

