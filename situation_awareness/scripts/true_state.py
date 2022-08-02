#!/usr/bin/env python3

import rospy
import sys
import numpy as np

from autonoship_simulation.msg import targetship_state
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

ownship = rospy.get_param("~ownship", "ownship")
targetship = rospy.get_param("~targetship", "targetship1")

print(sys.argv)
if len(sys.argv) == 3:
  ownship = sys.argv[1]
  targetship = sys.argv[2]
rospy.loginfo("frame: ", ownship, ", target: ", targetship)

own_twist = Twist()
own_speed = 0
target_twist = Twist()
target_speed = 0

truth_pub = rospy.Publisher(ownship + '/' + targetship + '/' + 'true_state', targetship_state, queue_size=1)
true_state = targetship_state()

def update_own_twist(msg):
  global own_twist
  own_twist = msg

def update_target_twist(msg):
  global target_twist
  target_twist = msg

def update_own_speed(msg):
  global own_speed
  own_speed = msg

def update_target_speed(msg):
  global target_speed
  target_speed = msg.data

def update_relevant():
  global true_state, own_twist, own_speed, target_twist, target_speed
  own = [own_twist.linear.x, own_twist.linear.y, own_twist.angular.z]
  target = [target_twist.linear.x, target_twist.linear.y, target_twist.angular.z]

  # rospy.loginfo(own)
  # rospy.loginfo(target)

  if not (np.linalg.norm(own) and np.linalg.norm(target)):
    return

  distance = np.sqrt((own[0] - target[0])**2 + (own[1] - target[1])**2)

  theta = np.arctan((target[0] - own[0])/(target[1] - own[1])) / np.pi * 180
  if (target[0] - own[0]) < 0:
    theta += 180
  bearing = own[2] - theta
  while bearing > 360:
    bearing -= 360
  while bearing < 0:
    bearing += 360

  course = 360 - (target[2] - own[2]) / np.pi * 180
  while course > 360:
    course -= 360
  while course < 0:
    course += 360

  speed = target_speed
  
  true_state.type = 'relavant'
  true_state.distance = distance
  true_state.bearing = bearing
  true_state.course = course
  true_state.speed = speed
  truth_pub.publish(true_state)


if __name__ == '__main__':
  rospy.init_node('true_state')

  rospy.Subscriber(ownship + '/twist', Twist, update_own_twist)
  rospy.Subscriber(targetship + '/twist', Twist, update_target_twist)
  rospy.Subscriber(ownship + '/vel_x', Float64, update_own_speed)
  rospy.Subscriber(targetship + '/vel_x', Float64, update_target_speed)
  
  rate = rospy.Rate(10)

  while not rospy.is_shutdown():
    update_relevant()
    rate.sleep()
