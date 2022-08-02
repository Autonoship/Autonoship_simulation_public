#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

import math
import numpy as np
import random
import itertools
import scipy.misc
import matplotlib.pyplot as plt
import os
from test_game import carmunk_placeholder as carmunk
import csv
import timeit
import pygame


####################################  NEW  ####################################
from PIL import Image
####################################  NEW  ####################################

####################################  NEW  ####################################


own_twist = None
target_twist = [None, None, None]

# action_pub = rospy.Publisher('own_action', Float64, queue_size=1)
zero_pub = rospy.Publisher('course_feedback', Float64, queue_size=1)
action_pub = rospy.Publisher('target_course', Float64, queue_size=1)

def update_own(msg):
    global own_twist
    own_twist = msg

def update_target1(msg):
    global target_twist
    target_twist[0] = msg

def update_target2(msg):
    global target_twist
    target_twist[1] = msg

def update_target3(msg):
    global target_twist
    target_twist[2] = msg

####################################  NEW  ####################################


# In[3]:

class gameEnv():
    def __init__(self):
        self.game_state = carmunk.GameState()
        self.actions = 29
        state, screen, valid_actions, course_reached, future_collision = self.reset()
        plt.imshow(state, interpolation = "nearest")

    def reset(self):
####################################  NEW  ####################################
        self.game_state.update_own(own_twist)
        self.game_state.update_target(target_twist[0])
####################################  NEW  ####################################
        _, surface, valid_actions, course_reached, future_collision = self.game_state.frame_step(0)
        screen = pygame.surfarray.pixels3d(surface)
        state = self.renderEnv(screen)
        self.state = state

        return state, screen, valid_actions, course_reached, future_collision

    def new_game(self):
        self.game_state = carmunk.GameState()

    def surf_to_pixel(surface):
        pixel_array_3d = pygame.surfarray.pixels3d(surface)
        return pixel_array_3d

    def moveChar(self, action):
####################################  NEW  ####################################
        self.game_state.update_own(own_twist)
        self.game_state.update_target(target_twist[0])
####################################  NEW  ####################################
        r, surface, valid_actions, course_reached, future_collision = self.game_state.frame_step((action))
        screen = pygame.surfarray.pixels3d(surface)
        return r, screen, valid_actions, course_reached, future_collision

    def renderEnv(self, screen):
        b = np.array(Image.fromarray(screen[:,:,0]).resize([84, 84]))
        c = np.array(Image.fromarray(screen[:,:,1]).resize([84, 84]))
        d = np.array(Image.fromarray(screen[:,:,2]).resize([84, 84]))
        # b = scipy.misc.imresize(screen[:,:,0], [84, 84, 1], interp = 'nearest')
        # c = scipy.misc.imresize(screen[:,:,1], [84, 84, 1], interp = 'nearest')
        # d = scipy.misc.imresize(screen[:,:,2], [84, 84, 1], interp = 'nearest')
        a = np.stack([b,c,d], axis=2)
        return a

    def step(self, action):
        r, screen, valid_actions, course_reached, future_collision = self.moveChar(action)
        state = self.renderEnv(screen)
        return state, r, screen, valid_actions, course_reached, future_collision


# In[4]:
####################################  NEW  ####################################
rospy.init_node('predict_action')

rospy.Subscriber('/ownship/twist', Twist, update_own)
rospy.Subscriber('/targetship1/twist', Twist, update_target1)
rospy.Subscriber('/targetship2/twist', Twist, update_target2)
rospy.Subscriber('/targetship3/twist', Twist, update_target3)
rate = rospy.Rate(1)

while not (own_twist and target_twist[0]):
    # rospy.loginfo(own_twist)
    # rospy.loginfo(target_twist)
    # rospy.loginfo("waiting for twist information...")
    rate.sleep()
####################################  NEW  ####################################

rospy.loginfo("targetship detected, initializing collision avoidance")
env = gameEnv()



# In[6]:

class experience_buffer():
    def __init__(self, buffer_size = 50000):
        self.buffer = []
        self.buffer_size = buffer_size

    def add(self,experience):
        if len(self.buffer) + len(experience) >= self.buffer_size:
            self.buffer[0:(len(experience)+len(self.buffer))-self.buffer_size] = []
        self.buffer.extend(experience)

    def sample(self,size):
        return np.reshape(np.array(random.sample(self.buffer,size)),[size,5])


# In[7]:

def processState(states):
    return np.reshape(states, [21168])




# In[9]:

batch_size = 32
update_freq = 8
y = .99
startE = 1
endE = 0.1
anneling_steps = 1000000
dom_period = 700000
num_episodes = 10000
pre_train_steps = 1000
max_epLength = 800
load_model = True
path = "./base"
load = "./base"
h_size = 512
tau = 0.001
belief = 0.3



# In[ ]:

myBuffer = experience_buffer()

e = startE
epsDrop = (startE - endE)/anneling_steps
beliefDrop = belief/dom_period

rList = []
total_steps = 0

if not os.path.exists(path):
    os.makedirs(path)


# In[ ]:

while not rospy.is_shutdown():
    rate.sleep()
    for i in range(num_episodes+1):
        episodeBuffer = experience_buffer()
        #Reset environment and get first new observation
        state, screen, valid_actions, course_reached, future_collision = env.reset()
        s = processState(state)
        d = False
        flag = False
        rAll = 0
        j = 0
        previous_action = 14
    
        #The Q-Network
        while j < max_epLength:
            rate.sleep()
            j += 1

            state, screen, valid_actions, course_reached, future_collision = env.reset()

            if j == 1:
                a = 14
            else:
                if True:  # if course_reached:
                    if future_collision:
                        a = valid_actions[0]
                        previous_action = a
                    else:
                        a = 30
                        previous_action = 30
                else:
                    a = previous_action
	    # course goal is from -70 to 70 deg, every 5 deg increment
            if a == 30:
                course_goal = 'base'
                print('go to the goal')
                theta = math.radians(env.game_state.own_rot_deg)
                x_0, y_0 = env.game_state.own_center_pos
                yg_local = -(env.game_state.x_g - x_0) * math.sin(theta) + (env.game_state.y_g - y_0) * math.cos(theta)
                xg_local = (env.game_state.x_g - x_0) * math.cos(theta) + (env.game_state.y_g - y_0) * math.sin(theta)
                angle_to_goal_deg = math.degrees(math.atan2(yg_local, xg_local))
                # if abs(angle_to_goal_deg) > 10:
                    # angle_to_goal_deg += np.sign(angle_to_goal_deg) * 70
                action_pub.publish(angle_to_goal_deg *np.pi / 180.0)
                zero_pub.publish(0.0)
            else:
                print('avoid collision')
                course_goal = -70.0 + 5 * a
                # print('At step ', j, 'course goal = ', course_goal, '---reached? ', course_reached)
                theta = env.game_state.own_rot_deg + 90
                angle_to_course_goal_deg = course_goal - theta
                # if abs(angle_to_course_goal_deg) > 10:
                    # angle_to_course_goal_deg += np.sign(angle_to_course_goal_deg) * 70
                action_pub.publish(angle_to_course_goal_deg *np.pi / 180.0)
                zero_pub.publish(0.0)


