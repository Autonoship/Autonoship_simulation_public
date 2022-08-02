#!/usr/bin/env python3

import rospy
import rospkg
import numpy as np
import cv2
import time

from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist

from GlobalPlanner import GlobalPlanner

def initialize_global_planner(global_planner: GlobalPlanner, name="GA_GP", chart_path=None, targetship_list=[]) -> bool:
  ok = global_planner.initialize(name=name, chart_path=chart_path)
  if targetship_list and name=="risk_aware_GP":  # placeholder
    ok = ok and global_planner.update_targetship(targetship_list)
   
  return ok

def reset_global_planner(global_planner: GlobalPlanner) -> bool:
  name = global_planner.name
  chart = global_planner.chart
  ok = initialize_global_planner(global_planner, name, chart, ([], []))
  return ok
  
def update_map(path: str):
  # placeholder
  pass
  
def update_targetship():
  # placeholder
  pass
  
def make_global_plan(global_planner, goal: PoseStamped):
  global own_state
  start = PoseStamped()
  start.pose.position.x = own_state[0]
  start.pose.position.y = own_state[1]
  start.pose.position.z = own_state[2]
  
  plan = []
  ok = global_planner.make_plan(start, goal, plan)
  
  return plan
  
def publish_plan():
  pass
  
# def get_map_from_SA():
#   pass
  
# def get_target_state_from_SA():
#   # placeholder
#   pass
  
# def get_own_state_from_SA():
#   pass

def update_state_cb(msg, arg):
  global own_state
  if arg == "x":
    own_state[0] = int(msg.data) / 10
  elif arg == "y":
    own_state[1] = int(35020 - msg.data) / 10  # for the cv2 coordinate, the origin locates at the top left corner
  else:  # if arg == "phi"
    phi = msg.data
    own_state[2] = 90 - phi / np.pi * 180 
    while own_state[2] >= 360:
      own_state[2] -= 360
    while own_state[2] < 0:
      own_state[2] += 360

def update_goal_cb(msg, goal):
  goal[0] = int(msg.linear.x) / 10
  goal[1] = int(35020 - msg.linear.y) / 10

def cv2_to_gazebo(plan):
  new_plan = []
  for x, y in plan:
    new_plan.append([10*x, 10*(3502 - y)])
  return new_plan

if __name__ == "__main__":
  rospy.init_node('path_planning')
  rate = rospy.Rate(1/600)

  directory = rospkg.RosPack().get_path('situation_awareness') + "/nautical_chart/"
  chart_path = directory + rospy.get_param('~chart_file')
  chart = cv2.imread(chart_path)

  # chart = get_map_from_SA()
  
  gx, gy = float(rospy.get_param('~goal_x')), float(rospy.get_param('~goal_y'))
  goal = [gx / 10, (35020 - gy) / 10]
  goal_sub = rospy.Subscriber("goal", Twist, update_goal_cb, goal)  # goal in cv2 frame
  
  plan = []
  plan_pub = rospy.Publisher("global_plan", Float64MultiArray, queue_size=1)
  
  own_state = [0, 3502, 90]  # x, y, course in cv2 coordinate
  
  own_x_sub = rospy.Subscriber('own_x', Float64, update_state_cb, callback_args="x")
  own_y_sub = rospy.Subscriber('own_y', Float64, update_state_cb, callback_args="y")
  own_course_sub = rospy.Subscriber('own_phi', Float64, update_state_cb, callback_args="phi")
  
  if not chart_path:
    rospy.logerr("No chart found! The path planning module has turned off.")
    assert(False)
    
  # own_state = get_own_state_from_SA()
  if not own_state:
    rospy.logerr("Failed to find the own state! The path planning module has shut down.")
    assert(False)
    
  targetship_list = []
  # targetship_list = get_target_state_from_SA()
   
  global_planner = GlobalPlanner()
  initialize_global_planner(global_planner, name="GA_GP", chart_path=chart_path, targetship_list=targetship_list)
  
  while True:
    plan = []
    goal_pose_stamped = PoseStamped()
    goal_pose_stamped.pose.position.x = goal[0]
    goal_pose_stamped.pose.position.y = goal[1]
    t1 = time.time()
    plan = make_global_plan(global_planner, goal_pose_stamped)
    t2 = time.time()
    
    if not plan:
      rospy.logerr("No feasible plan found! The path planning module has shut down.")
      break
    else:
      rospy.logdebug("Have made a new plan. The planning takes " + str(t2 - t1) + " seconds.")
      # rospy.logdebug(str(plan))
      
    plan_gazebo_frame = cv2_to_gazebo(plan)
    plan_msg = Float64MultiArray()
    for x, y in plan_gazebo_frame:
      plan_msg.data.append(x)
      plan_msg.data.append(y)
    plan_pub.publish(plan_msg)
    
    rate.sleep()
