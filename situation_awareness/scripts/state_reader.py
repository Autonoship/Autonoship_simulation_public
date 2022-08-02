#!/usr/bin/env python3

import rospy

import matplotlib.pyplot as plt 
import numpy as np

from autonoship_simulation.msg import targetship_state
from std_msgs.msg import Float64
from std_msgs.msg import String
from geometry_msgs.msg import Twist

ship_dict = ['r', 'g', 'b']
state = [None, None, None]

def draw_circle((x, y), r, line_type, ax):
  rads = np.arange(-np.pi, np.pi + 0.1, 0.1)
  ax.plot(r * np.cos(rads) + x, r * np.sin(rads) + y, line_type)

def draw_arrow((x, y), r, line_type, ax):
  rads = np.arange(-np.pi, np.pi + 0.1, 0.1)
  ax.plot(r * np.cos(rads) + x, r * np.sin(rads) + y, line_type)

def update_canvas(msg):
  # rospy.loginfo(msg.type)
  if msg.type:
    state[int(msg.type[-1]) - 1] = msg
    # rospy.loginfo(msg.type)


if __name__ == '__main__':
  rospy.init_node('state_reader')

  plt.ion()
  fig = plt.figure()
  ax = fig.add_subplot(111)
  draw_circle((0, 0), 1, 'b--', ax)
  for i in range(1, 3):
    draw_circle((0, 0), 2*i, 'b--', ax)
  fig.canvas.draw()

  rospy.Subscriber('ownship/targetship1/state', targetship_state, update_canvas)
  rospy.Subscriber('ownship/targetship2/state', targetship_state, update_canvas)
  rospy.Subscriber('ownship/targetship3/state', targetship_state, update_canvas)
  rate = rospy.Rate(10)

  while not rospy.is_shutdown():
    rate.sleep()
    # rospy.loginfo(len(msg.models))
    ax.cla()
    ax = fig.add_subplot(111)
    draw_circle((0, 0), 1, 'b--', ax)
    for i in range(1, 3):
      draw_circle((0, 0), 2*i, 'b--', ax)
    fig.canvas.draw()
    for i in range(len(state)):
      if state[i]:
        bearing = state[i].bearing
        distance = state[i].distance
        course = state[i].course
        speed = state[i].speed

        # rospy.loginfo(distance)
        x = np.sin(bearing*np.pi/180) * distance
        y = np.cos(bearing*np.pi/180) * distance
        dx = speed * np.sin(course*np.pi/180) * 0.02
        dy = speed * np.cos(course*np.pi/180) * 0.02
        # rospy.loginfo(x)
        # rospy.loginfo(y)
        ax.arrow(x, y, dx, dy)

        fig.canvas.draw()
