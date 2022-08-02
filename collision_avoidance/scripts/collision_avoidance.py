#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist

from LocalPlanner import LocalPlanner

def initialize_local_planner(local_planner: LocalPlanner, name="risk_aware_GP", chart, targetship_list=[]) -> bool:
  ok = local_planner.initialize(name=name, chart=chart)
  if targetship_list and name=="risk_aware_GP":
    ok = ok and local_planner.update_targetship(targetship_list)
   
  return ok

  
def reset_local_planner(local_planner: LocalPlanner) -> bool:
  name = local_planner.name
  chart = local_planner.chart
  ok = initialize_local_planner(local_planner, name, chart, ([], []))
  return ok
  
def update_map(path: str):
  pass
  
def update_targetship():
  pass
  
def make_local_plan(waypoint: PoseStamped) -> bool:
  pass
  
def publish_plan():
  pass
  
def get_map_from_SA():
  pass
  
def get_target_state_from_SA():
  pass
  
def get_own_state_from_SA():
  pass

def update_waypoint_cb(msg, waypoint):
  waypoint[0] = msg.linear.x
  waypoint[1] = msg.linear.y

if __name__ == "__main__":
  rospy.init_node('path_planning')
  rate = rospy.Rate(60000)

  chart = None
  chart = get_map_from_SA()
  
  waypoint = [0, 0]
  waypoint_sub = rospy.Subscriber("waypoint", Twist, update_waypoint_cb, waypoint)
  
  # placeholder for action publisher
  # action = []
  action_pub = rospy.Publisher("ownship/u1", Float64, queue_size=1)
  
  while True:
    action = []
    
    # placeholder. The current collision avoidance module only deals with the open-sea area.
    if not chart:
      pass
    
    own_state = get_own_state_from_SA()
    if not own_state:
      rospy.logerr("Failed to find the own state! The collision avoidance module has shut down.")
      break
    
    targetship_list = get_target_state_from_SA()
    
    local_planner = LocalPlanner()
    initialize_local_planner(local_planner, name="CA_RL", chart, targetship_list))
    
    action = make_local_plan(waypoint)
    if not action:
      rospy.logerr("Failed to find a action! The collision avoidance module has shut down.")
      break
      
    action_msg = Float64()
    action_msg.data = plan
    action_pub.publish(action_msg)
    
    rate.sleep()
