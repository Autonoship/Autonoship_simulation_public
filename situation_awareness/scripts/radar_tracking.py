#!/usr/bin/env python3

import rospy

import numpy as np

from usv_gazebo_plugins.msg import LogicalCameraImage
from std_msgs.msg import Float64
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from autonoship_simulation.msg import radar_tracking

pub_0 = rospy.Publisher('ownship/tracking', radar_tracking, queue_size=1)
pub_1 = rospy.Publisher('targetship1/tracking', radar_tracking, queue_size=1)
pub_2 = rospy.Publisher('targetship2/tracking', radar_tracking, queue_size=1)
pub_3 = rospy.Publisher('targetship3/tracking', radar_tracking, queue_size=1)

ship_dict = {'ownship': pub_0, 'targetship1': pub_1, 'targetship2': pub_2, 'targetship3': pub_3} 
tracking = {'ownship': None, 'targetship1': None, 'targetship2': None, 'targetship3': None}
bearing_min = {'ownship': None, 'targetship1': None, 'targetship2': None, 'targetship3': None}
bearing_max = {'ownship': None, 'targetship1': None, 'targetship2': None, 'targetship3': None}

yaw_self = 0

nm = rospy.get_param("~nm", True)

def update_yaw(msg):
  global yaw_self
  # rospy.loginfo(yaw_self)
  yaw_self = msg.data
  
  

def update_tracking(msg):
  detected = []
  if msg.models:
    # rospy.loginfo(msg.models)
    # rospy.loginfo(len(msg.models))
    own_x = msg.pose.position.x
    own_y = msg.pose.position.y
    own_orientation = msg.pose.orientation
    orientation_list = [own_orientation.x, own_orientation.y, own_orientation.z, own_orientation.w]
    bearing = 180/3.1415926 * (yaw_self - euler_from_quaternion(orientation_list)[2])

    while bearing > 360:
      bearing -= 360
    while bearing < 0:
      bearing += 360

    

    for i in range(len(msg.models)):
      # if ship_dict.has_key(msg.models[i].type):
      if msg.models[i].type in ship_dict:
        ship = msg.models[i].type
        detected.append(ship)
        if bearing_min[ship] == None:
          tracking[ship] = radar_tracking()
          tracking[ship].type = ship



          # x = own_x - msg.models[i].pose.position.x
          # y = own_y - msg.models[i].pose.position.y
          # tracking[ship].distance = np.sqrt(x**2 + y**2)


############### need further attention ####################

          x = msg.models[i].pose.position.x
          if not tracking[ship].distance:
            tracking[ship].distance = x
          if tracking[ship].distance < x:
            tracking[ship].distance = x
            bearing_min[ship] = bearing

############### need further attention ####################

          bearing_min[ship] = bearing
        bearing_max[ship] = bearing

  for ship in ship_dict.keys():
    if tracking[ship] and ship not in detected:
      if nm:
        tracking[ship].distance /= 1852
      # if bearing_max[ship] < 60 and bearing_min[ship] > 300:
      #   bearing_max[ship] += 360
      # tracking[ship].bearing = (bearing_max[ship] + bearing_min[ship]) / 2.0
      # if tracking[ship].bearing > 360:
      #   tracking[ship].bearing -= 360

      tracking[ship].bearing = bearing_min[ship]
      ship_dict[ship].publish(tracking[ship])
      bearing_min[ship] = None
      bearing_max[ship] = None
      tracking[ship] = None
        

if __name__ == '__main__':
  rospy.init_node('radar_tracking')

  rospy.Subscriber('logical_camera', LogicalCameraImage, update_tracking)
  rospy.Subscriber('yaw', Float64, update_yaw)
  
  rate = rospy.Rate(10)

  while not rospy.is_shutdown():
    rate.sleep()
