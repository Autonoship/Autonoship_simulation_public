#!/usr/bin/env python3

import rospy

import matplotlib.pyplot as plt 
import numpy as np

from autonoship_simulation.msg import radar_tracking
from std_msgs.msg import Float64
from std_msgs.msg import String
from geometry_msgs.msg import Twist

ship_dict = ['r', 'g', 'b']
tracking = [None, None, None]

def draw_circle((x, y), r, line_type, ax):
  rads = np.arange(-np.pi, np.pi + 0.1, 0.1)
  ax.plot(r * np.cos(rads) + x, r * np.sin(rads) + y, line_type)

def update_canvas(msg):
  # rospy.loginfo(msg.type)
  if msg.type:
    tracking[int(msg.type[-1]) - 1] = msg
    # rospy.loginfo(msg.type)


if __name__ == '__main__':
  rospy.init_node('radar_reader')

  plt.ion()
  fig = plt.figure()
  ax = fig.add_subplot(111)
  draw_circle((0, 0), 1, 'b--', ax)
  for i in range(1, 3):
    draw_circle((0, 0), 2*i, 'b--', ax)
  fig.canvas.draw()

  rospy.Subscriber('ownship/targetship1/tracking', radar_tracking, update_canvas)
  rospy.Subscriber('ownship/targetship2/tracking', radar_tracking, update_canvas)
  rospy.Subscriber('ownship/targetship3/tracking', radar_tracking, update_canvas)
  rate = rospy.Rate(10)

  while not rospy.is_shutdown():
    rate.sleep()
    # rospy.loginfo(len(msg.models))
    for i in range(len(tracking)):
      if tracking[i]:
        bearing = tracking[i].bearing
        distance = tracking[i].distance
        # rospy.loginfo(distance)
        x = np.sin(bearing*np.pi/180) * distance
        y = np.cos(bearing*np.pi/180) * distance
        # rospy.loginfo(x)
        # rospy.loginfo(y)
        draw_circle((x, y), 0.1, ship_dict[i], ax)

        fig.canvas.draw()
