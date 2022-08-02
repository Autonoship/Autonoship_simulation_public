#!/usr/bin/env python3

import rospy
import rospkg
import numpy as np
import cv2

from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist

def draw_path(chart, path, color=None, width=None):
    if not color:
        color = (0, 255, 0)
    if not width:
        width = 5
    for i in range(len(path) - 1):
        start_point = path[i]
        end_point = path[i+1]
        cv2.line(chart, start_point, end_point, color, width)
    return chart


def draw_ship(chart, pos, course_in_degree, scale, color):
    x, y = pos
    phi = (course_in_degree - 90) * np.pi / 180
    ship_contour = scale * np.matrix([[30, 0], 
                              [10, -10], 
                              [-30, -10], 
                              [-30, 10], 
                              [10, 10]]).T
    rot = np.matrix([[np.cos(phi), -np.sin(phi)], 
                     [np.sin(phi), np.cos(phi)]])
    ship_contour = np.round(np.matmul(rot, ship_contour)).astype(int)
    
    for i in range(5):
        ship_contour[0, i] += x
        ship_contour[1, i] += y
        
    cv2.fillPoly(chart, pts=[ship_contour.T], color=color)
    return chart


def draw_goal(chart, pos, scale=1, color=(0, 255, 0)):
    x, y = pos
    contour = scale * np.matrix([[20, -20], 
                                [-20, -20], 
                                [-20, 20], 
                                [20, 20]]).T
    
    for i in range(4):
        contour[0, i] += x
        contour[1, i] += y
        
    cv2.fillPoly(chart, pts=[contour.T], color=color)
    return chart

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
  goal[1] = int((35020 - msg.linear.y) / 10)

def update_plan_cb(msg):  # use cv2 frame
  global plan
  plan = []
  p = msg.data
  for i in range(0, len(p), 2):
    x = int(p[i] / 10)
    y = int((35020 - p[i+1]) / 10)
    plan.append([x, y])



if __name__ == "__main__":
  rospy.init_node('show_own_path')
  rate = rospy.Rate(1)

  directory = rospkg.RosPack().get_path('situation_awareness') + "/nautical_chart/"
  chart_path = directory + rospy.get_param('~chart_file')
  chart = cv2.imread(chart_path)

  # chart = get_map_from_SA()
  
  gx, gy = float(rospy.get_param('~goal_x')), float(rospy.get_param('~goal_y'))
  goal = [gx / 10, (35020 - gy) / 10]
  goal_sub = rospy.Subscriber("ownship/goal", Twist, update_goal_cb, goal)  # goal in cv2 frame
  
  plan = []
  plan_sub = rospy.Subscriber("ownship/global_plan", Float64MultiArray, update_plan_cb)
  
  own_state = [0, 3502, 90]  # x, y, course in cv2 coordinate
  
  own_x_sub = rospy.Subscriber('ownship/own_x', Float64, update_state_cb, callback_args="x")
  own_y_sub = rospy.Subscriber('ownship/own_y', Float64, update_state_cb, callback_args="y")
  own_course_sub = rospy.Subscriber('ownship/own_phi', Float64, update_state_cb, callback_args="phi")
    
  targetship_list = []
  # targetship_list = get_target_state_from_SA()

  
  while True:
    print(plan)
    image = chart.copy()
    
    draw_path(image, plan)
    image = draw_ship(image, [own_state[0], own_state[1]], own_state[2], scale=2, color=[0, 0, 255])
    image = draw_goal(image, goal)
    
    image = cv2.resize(image, [image.shape[1] // 10, image.shape[0] // 10])
    cv2.imshow("image", image)
    cv2.waitKey(1)

    
    rate.sleep()
