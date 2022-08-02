#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import String
from geometry_msgs.msg import Twist

key_mapping = {'w': [0, 1], 'a': [1, 0], 's': [0, 0], 
               'd': [-1, 0], 'q': [1, 1], 'e': [-1, 1]}

u1_last_t = Float64()
u2_last_t = Float64()

u1_pub = rospy.Publisher('u1', Float64, queue_size=1)
u2_pub = rospy.Publisher('u2', Float64, queue_size=1)

def keys_cb(msg, pub_vec):
  global u1_last_t
  global u2_last_t
  # global u1_pub
  # global u2_pub
  u1_pub = pub_vec[0]
  u2_pub = pub_vec[1]

  scale = 1000.0
  if len(msg.data) == 0 or not key_mapping.has_key(msg.data[0]):
    return
  rudder = key_mapping[msg.data[0]][0]
  thrust = key_mapping[msg.data[0]][1]
  u2_last_t.data = thrust * scale /10
  u1_last_t.data = rudder * scale
  u2_pub.publish(u2_last_t)
  u1_pub.publish(u1_last_t)

if __name__ == '__main__':
  rospy.init_node('keys_to_rudder')

  rospy.Subscriber('keys', String, keys_cb, (u1_pub, u2_pub))
  rate = rospy.Rate(10)

  while not rospy.is_shutdown():
    u1_pub.publish(u1_last_t)
    u2_pub.publish(u2_last_t)
    rate.sleep()
