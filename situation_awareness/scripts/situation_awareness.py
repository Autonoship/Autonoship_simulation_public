#!/usr/bin/env python3
import rospy
import cv2

# from nav_msgs import OccupancyGrid
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from autonoship_simulation.msg import targetship_state


class OwnState:
  def __init__(self, x=0, y=0, phi=0, dphi=0, u=0):
    self.x = x
    self.y = y
    self.phi = phi
    self.dphi = dphi  # placeholder. The angular speed is controlled by the "navigation control module", and is not considered in the current path planning algorithm.
    self.u = u
    

class Situation:
  def __init__(self, chart):
    self.map = chart
    self.target_ship_list = {}
    self.own_state = OwnState()
    
  def updateShipState(self, ship_idx, state_msg):
    if ship_idx > 0:  # if targetship
      # then update the state of the targetship
      if ship_idx not in self.target_ship_list:
        self.target_ship_list[ship_idx] = OwnState()
        
      ###  place holder  ###
      self.target_ship_list[ship_idx].x = 0
      self.target_ship_list[ship_idx].y = 0
      self.target_ship_list[ship_idx].phi = 0
      
    else:  # if ownship
      # then update the state of the ownship
      
      if isinstance(state_msg, Twist):
        self.own_state.x = state_msg.linear.x
        self.own_state.y = state_msg.linear.y
        self.own_state.phi = state_msg.angular.z
      else:
        self.own_state.u = state_msg.data
    
  def updateMap(self, chart, map_extent):  # placeholder
    # transfer the coordinate of states of ships here
    pass
    
  def publishAll(self):
    pass
    
        
def state_cb(msg, info):  # info = (situation, name)
  SA = info[0]
  name = info[1]
  if name[:10] == "targetship":
    idx = int(name[10:])
    SA.updateShipState(idx, msg)
    
  elif name == "ownship":
    idx = -1
    SA.updateShipState(idx, msg)
    
  else:  # if name == "None"
    # placeholder for general purpose targetship identification and state update. Does not know the exact number of targetships 
    pass

def map_cb(msg, situation):  # placeholder
  # update map info, calculate coordinate transformation
  pass


if __name__ == '__main__':
  rospy.init_node('situation_awareness')
  rate = rospy.Rate(10)
  
  chart = cv2.imread("../nautical_chart/res_100_lon_-122.67_-122.22_lat_37.54_38.17.jpg")
  situation = Situation(chart)
        
  target1_state_sub = rospy.Subscriber('targetship1/state', targetship_state, state_cb, callback_args=(situation, "targetship1"))
  target2_state_sub = rospy.Subscriber('targetship2/state', targetship_state, state_cb, callback_args=(situation, "targetship2"))
  target3_state_sub = rospy.Subscriber('targetship3/state', targetship_state, state_cb, callback_args=(situation, "targetship3"))
  
  general_target_state_sub = rospy.Subscriber('targetship/state', targetship_state, state_cb, callback_args=(situation, "None"))  # placeholder, not active in the simulation test
  
  own_twist_sub = rospy.Subscriber('twist', Twist, state_cb, callback_args=(situation, "ownship"))  # twist info from Gazebo simulator
  own_vel_sub = rospy.Subscriber('vel_x', Float64, state_cb, callback_args=(situation, "ownship"))
  
#   map_sub = rospy.Subscriber('chart', OccupancyGrid, map_cb, situation)
  
  pub_own_x = rospy.Publisher('own_x', Float64, queue_size=1)
  pub_own_y = rospy.Publisher('own_y', Float64, queue_size=1)
  pub_own_phi = rospy.Publisher('own_phi', Float64, queue_size=1)
  pub_own_u = rospy.Publisher('own_u', Float64, queue_size=1)
  
    
  while not rospy.is_shutdown():
    pub_own_x.publish(situation.own_state.x)
    pub_own_y.publish(situation.own_state.y)
    pub_own_phi.publish(situation.own_state.phi)
    pub_own_u.publish(situation.own_state.u)
    rate.sleep()


