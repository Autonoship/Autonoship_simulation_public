#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid

from BaseLocalPlanner import BaseLocalPlanner

class LocalPlanner(BaseLocalPlanner):
  def __init__(self, targetship_list=[], targetship_intention=[]):
    self.name = None
    self.chart = None
    self.targetship_list = targetship_list
    self.targetship_intention = targetship_intention
    
  def make_plan(self, start: PoseStamped, goal: PoseStamped, plan) -> bool:
    self.make_plan_under_risk(start, goal, self.targetship_intention, plan)
    return True
  
  def initialize(self, name: str, chart: OccupancyGrid) -> bool:
    self.name = name
    self.chart = chart
    return True  
   
  def update_targetship(targetship_list=[]) -> bool:
    self.targetship_list = targetship_list
    
    # infer the targetship_intention here
    self.targetship_intention = []  #################################
    
    return True
  
  def make_plan_under_risk(self, start, goal, targetship_intention, plan) -> bool:
    return True
