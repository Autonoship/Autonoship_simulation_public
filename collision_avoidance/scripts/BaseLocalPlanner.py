#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid

class BaseLocalPlanner:
  def make_plan(self, start: PoseStamped, goal: PoseStamped, plan) -> bool:
    plan = []
    return True
  
  def initialize(self, name: str, map: OccupancyGrid) -> bool:
    return True
  
  
