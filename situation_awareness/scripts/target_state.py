#!/usr/bin/env python3

import rospy

import numpy as np

from autonoship_simulation.msg import radar_tracking
from autonoship_simulation.msg import targetship_state

pub = rospy.Publisher('state', targetship_state, queue_size=1)

nm = rospy.get_param("~nm", True)

dt = 60.0 / 24.0  # rpm: 24
x = None
y = None
phi = None
v = None
state_ = np.matrix([x, y, phi, v]).T

P = np.matrix([[100, 0, 0, 0], 
               [0, 100, 0, 0], 
               [0, 0, 10, 0], 
               [0, 0, 0, 10]])

F = np.matrix([[1, 0, 0, 0], 
               [0, 1, 0, 0], 
               [0, 0, 1, 0], 
               [0, 0, 0, 1]])

# Q = np.matrix([[ 4.7841776,  3.7841776, -0.1929165,  0.025    ],
#                [ 3.7841776,  4.7841776, -0.1929165,  0.025    ],
#                [-0.1929165, -0.1929165,  0.01     ,  0.       ],
#                [ 0.025    ,  0.025    ,  0.       ,  0.01     ]])



# Q = np.matrix([[2.21, -1.17, 0.11, 0.014], 
#                [-1.17, 2.21, -0.11, 0.014], 
#                [0.11, -0.11, 0.01, 0], 
#                [0.014, 0.014, 0, 0.01]])  # need further testing

# Q = np.matrix([[4.2, 0.8, 0.1, 1.414], 
#                [0.8, 4.2, -0.1, 1.414], 
#                [0.1, -0.1, 0.01, 0], 
#                [1.414, 1.414, 0, 1]])  # need further testing

Q = np.matrix([[1.    , 0.    , 0.    , 0.    ],
               [0.    , 1.    , 0.    , 0.    ],
               [0.    , 0.    , 0.0001, 0.    ],
               [0.    , 0.    , 0.    , 0.01  ]])


H = np.matrix([[1, 0, 0, 0], 
               [0, 1, 0, 0]])

R = np.matrix([[10000, 0], 
               [0, 10000]])

def ekf(msg):
  # rospy.loginfo(msg.models)
  # rospy.loginfo(len(msg.models))
  global dt, x, y, phi, v, state_
  global P, F, Q, H, R

  shiptype = msg.type
  bearing = msg.bearing  # degrees
  distance = msg.distance
  if nm:
    distance = distance * 1852  # meter

  if not phi:

    
    if not x:
      x = np.cos(bearing*np.pi/180) * distance
      y = - np.sin(bearing*np.pi/180) * distance
      return

    dx = np.cos(bearing*np.pi/180) * distance - x + dt*7.56
    dy = - np.sin(bearing*np.pi/180) * distance - y
    phi = np.arctan(- dy/dx)
    
    if phi < 0:
      phi += np.pi
    if dy > 0:
      phi += np.pi
    
    
    # phi = np.pi

    x = np.cos(bearing*np.pi/180) * distance
    y = - np.sin(bearing*np.pi/180) * distance
    # phi = 0
    v = 15 * 0.514444  # m/s

    target_state = targetship_state()
    target_state.type = shiptype
    target_state.bearing = bearing
    target_state.distance = distance
    if nm:
      target_state.distance = target_state.distance / 1852  # meter
    target_state.course = phi / np.pi * 180
    target_state.speed = v / 0.514444  # knots
    pub.publish(target_state)

    F = np.matrix([[1, 0, -dt*np.sin(phi)*v, dt*np.cos(phi)], 
                  [0, 1, -dt*np.cos(phi)*v, -dt*np.sin(phi)], 
                  [0, 0, 1, 0], 
                  [0, 0, 0, 1]])
    state_ = np.matrix([x, y, phi, v]).T  # [m, m, rad, m/s].T
    rospy.loginfo("state_ = %s", state_)

  else:
    P = F.dot(P).dot(F.T) + Q  # prediction
    state_[0] = state_[0] + dt*state_[3]*np.cos(state_[2]) - dt*7.56  # 15*0.514444  # prediction
    state_[1] = state_[1] - dt*state_[3]*np.sin(state_[2])  # prediction
    if state_[3] < 0:
      state_[3] = -state_[3]
      state_[2] += np.pi
    state_[2] = np.mod(state_[2], 2*np.pi)
    phi = float(state_[2])
    v = float(state_[3])

    F = np.matrix([[1, 0, -dt*np.sin(phi)*v, dt*np.cos(phi)], 
                   [0, 1, -dt*np.cos(phi)*v, -dt*np.sin(phi)], 
                   [0, 0, 1, 0], 
                   [0, 0, 0, 1]])

    R = np.matrix([[max(100, (0.01*distance*np.cos(bearing*np.pi/180))**2), - 100*np.cos(bearing*np.pi/180)*np.sin(bearing*np.pi/180)], 
                   [- 100*np.cos(bearing*np.pi/180)*np.sin(bearing*np.pi/180), max(100, (0.01*distance*np.sin(bearing*np.pi/180))**2)]])
    # R = np.matrix([[0.01, 0], 
    #                [0, 0.01]])

    rospy.loginfo("bearing = %s", bearing)
    z = np.matrix([[np.cos(bearing*np.pi/180) * distance], 
                   [- np.sin(bearing*np.pi/180) * distance]])
    rospy.loginfo("z = %s", z)

    K = P.dot(H.T).dot(np.linalg.inv(H.dot(P).dot(H.T) + R))

    # rospy.loginfo("state_ = %s", state_)
    # rospy.loginfo("K = %s", K)
    # rospy.loginfo("z = %s", z)
    # rospy.loginfo("H.dot(state_) = %s", H.dot(state_))
    state_ = state_ + K.dot(z - H.dot(state_))
    P = (np.matrix(np.eye(4)) - K.dot(H)).dot(P)

    # rospy.loginfo("state_ = %s", state_)
    bearing = np.arctan(-state_[1]/state_[0])
    distance = np.sqrt(state_[0]**2 + state_[1]**2)
    # rospy.loginfo(bearing)

    if bearing < 0:
      bearing += np.pi
    if state_[1] > 0:
      bearing += np.pi

    target_state = targetship_state()
    target_state.type = shiptype
    target_state.bearing = bearing / np.pi * 180
    target_state.distance = distance 
    if nm:
      target_state.distance = target_state.distance / 1852  # nm
    target_state.course = state_[2] / np.pi * 180  # degrees
    target_state.speed = state_[3] / 0.514444  # knots
    pub.publish(target_state)
        

if __name__ == '__main__':
  rospy.init_node('target_state')

  rospy.Subscriber('tracking', radar_tracking, ekf)
  
  rate = rospy.Rate(10)

  while not rospy.is_shutdown():
    rate.sleep()
