#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid

from BaseGlobalPlanner import BaseGlobalPlanner

import rospy
import pandas as pd
import cv2
import random
from os import walk
import numpy as np
import matplotlib.pyplot as plt
import pygad
# import multiprocessing
# from multiprocessing import Pool
import skfmm
# from mpl_toolkits.axes_grid1 import make_axes_locatable

class GlobalPlanner(BaseGlobalPlanner):
  def __init__(self, targetship_list=[], targetship_intention=[]):
    self.name = None
    self.chart = None
    self.chart_path = None
    self.fmm_d = None
    self.old_goal_pos = None
    self.kernel_size = 25
    self.GA_generations = 200
    self.dilate_kernel = np.ones((self.kernel_size, self.kernel_size), np.uint8)
    self.targetship_list = targetship_list
    self.targetship_intention = targetship_intention
    
  def make_plan(self, start: PoseStamped, goal: PoseStamped, plan) -> bool:
    start_pos = [start.pose.position.x, start.pose.position.y]
    start_COG = start.pose.position.z  # note, the z here is the start course angle
    goal_pos = [goal.pose.position.x, goal.pose.position.y]
    
    chart = self.chart
    if self.old_goal_pos != goal_pos:
      chart_grey = cv2.imread(self.chart_path, cv2.IMREAD_GRAYSCALE)
      chart_grey = cv2.dilate(chart_grey, self.dilate_kernel, iterations=1)
      fmm_img = chart_grey.copy().astype(int)
      mask = (fmm_img > 0)

      phi = - np.ones_like(fmm_img)
      phi[int(goal_pos[1]), int(goal_pos[0])] = 1
      phi = np.ma.MaskedArray(phi, mask)

      self.fmm_d = skfmm.distance(phi, dx=1)
      self.old_goal_pos = goal_pos
      
    # define functions for GA
    def GA_sol_to_path(solution):
      x, y = start_pos
      xg, yg = goal_pos
    
      path = [[x, y]]
      reached = False
      for d in solution:
        direction = [np.sin(d*np.pi/180), -np.cos(d*np.pi/180)]
        new_x, new_y = x + int(l*direction[0]), y + int(l*direction[1])
        path.append([new_x, new_y])
        
        if distance(new_x, new_y, xg, yg) < 300:  # if reached
          break
        
        x, y = new_x, new_y
        
      return path

    def xy_in_chart(new_x, new_y):
      if (0 <= new_y < chart.shape[0] and 0 <= new_x < chart.shape[1]):
        return True
      else:
        return False

    def fitness_func(solution, solution_idx):
      reach_threshold = 300
    
      x, y = start_pos
      x, y = int(x), int(y)
      xg, yg = goal_pos
    
      count = 0
      fitness = 0
      in_collision = False
      course = start_COG
      
      for idx, d in enumerate(solution):
        direction = [np.sin((d+course)*np.pi/180), -np.cos((d+course)*np.pi/180)]
        course += d
        new_x, new_y = x + int(l*direction[0]), y + int(l*direction[1])
        
        if not xy_in_chart(new_x, new_y):
          in_collision = True
        else:
          for a in [_/10 for _ in range(1, 11)]:
            xm, ym = int((1-a)*x + a*new_x), int((1-a)*y + a*new_y)
            if chart[ym][xm][0] > 0:
              in_collision = True
              break
            
        if in_collision:
          break
        
        count += 1
        if idx > 0:  # steering cost
          fitness += - 500 * (1 - np.cos(abs(d) * np.pi/180) ** 7)
        else:
          fitness += - 500 * (1 - np.cos(abs(d) * np.pi/180) ** 7)
    
        x, y = new_x, new_y
        if distance(x, y, xg, yg) < reach_threshold:  # if reached
            break
    
      fitness += - count * l
      if (not self.fmm_d[y][x]):
        fitness += -100000
      else:
        fitness += 2 * self.fmm_d[y][x]
    
      return fitness

    def GA_sol_to_path(solution):
      x, y = start_pos
      xg, yg = goal_pos
    
      path = [[x, y]]
      reached = False
      course = start_COG
      for d in solution:
        direction = [np.sin((d+course)*np.pi/180), -np.cos((d+course)*np.pi/180)]
        course += d
        new_x, new_y = x + int(l*direction[0]), y + int(l*direction[1])
        path.append([new_x, new_y])
        
        if distance(new_x, new_y, xg, yg) < 300:  # if reached
          break
        
        x, y = new_x, new_y
        
      return path

    def on_fitness(GA, fit_val):
      fitness_values.append(np.max(fit_val))

    def plan_with_GA(seed, on_fit=None):
      random.seed(seed)
      np.random.seed(seed)
      ga_instance = pygad.GA(num_generations=num_generations,
                           num_parents_mating=num_parents_mating,
                           fitness_func=fitness_function,
                           sol_per_pop=sol_per_pop,
                           num_genes=num_genes,
                           gene_space=gene_space,
                           init_range_low=init_range_low,
                           init_range_high=init_range_high,
                           parent_selection_type=parent_selection_type,
                           keep_parents=keep_parents,
                           K_tournament=K_tournament,
                           crossover_type=crossover_type,
                           crossover_probability=crossover_probability,
                           mutation_type=mutation_type,
                           mutation_probability=mutation_probability,
                           random_mutation_min_val=random_mutation_min_val,
                           random_mutation_max_val=random_mutation_max_val)
    
      ga_instance.run()
      solution, solution_fitness, solution_idx = ga_instance.best_solution()
      return(solution_fitness, solution)

    # parameters of GA
    fitness_function = fitness_func
    degree_step = 5
    degree_range = 40
    num_genes = 25
    l = 300  # step size in pixels
    gene_space = list(range(-degree_range, degree_range+1, degree_step))
    init_range_low = -degree_range
    init_range_high = degree_range
    parent_selection_type = "tournament"
    keep_parents = -1
    random_mutation_min_val = -degree_range
    random_mutation_max_val = degree_range
    
    num_generations = self.GA_generations
    sol_per_pop = 400
    crossover_type = "single_point"
    crossover_probability = 0.9
    mutation_type = "random"
    mutation_probability = 0.1
    K_tournament = 10
    num_parents_mating = 16

    # pool = Pool(processes=16)
    fit, solution = plan_with_GA(0, on_fitness)
    # rospy.logerr("fitness value: " + str(fit))
    
    # pool.close()
    # pool.join()

    path = GA_sol_to_path(solution)
    # rospy.logerr("solution: " + str(solution))
    for x, y in path:
      plan.append([x, y])

    return True
  
  def initialize(self, name: str, chart_path: str) -> bool:
    self.name = name
    self.chart = cv2.imread(chart_path)
    self.chart = cv2.dilate(self.chart, self.dilate_kernel, iterations=1)
    self.chart_path = chart_path
    
    return True  
   
  def update_targetship(targetship_list=[]) -> bool:  # placeholder
    self.targetship_list = targetship_list
    
    # infer the targetship_intention here (placeholder)
    self.targetship_intention = []  #################################
    
    return True
  
  def make_plan_under_risk(self, start, goal, targetship_intention, plan) -> bool:
    # placeholder
    return True
    
    
def distance(x0, y0, x1, y1):
    return np.sqrt((x0-x1)**2 + (y0-y1)**2)

def get_degree(x0, y0, x1, y1):
    dx, dy = x1 - x0, y1 - y0
    length = distance(x0, y0, x1, y1)
    theta = np.arccos(dx / length)
    if dy < 0:
        theta += 2 * np.pi
    
    degree = theta * 180 / np.pi + 90
    while degree > 360:
        degree -= 360
    while degree < 0:
        degree += 0
    return degree
