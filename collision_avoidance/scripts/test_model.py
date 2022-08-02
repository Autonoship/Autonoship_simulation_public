
# coding: utf-8

# In[1]:

import numpy as np
import random
import itertools
import scipy.misc
import matplotlib.pyplot as plt
import tensorflow as tf
import tensorflow.contrib.slim as slim
import os
from test_game import carmunk
import csv
import timeit
import pygame



# In[3]:

class gameEnv():
    def __init__(self):
        self.game_state = carmunk.GameState()
        self.actions = 29
        state, screen, valid_actions, course_reached, future_collision = self.reset()
        plt.imshow(state, interpolation = "nearest")

    def reset(self):

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
        r, surface, valid_actions, course_reached, future_collision = self.game_state.frame_step((action))
        screen = pygame.surfarray.pixels3d(surface)
        return r, screen, valid_actions, course_reached, future_collision

    def renderEnv(self, screen):
        b = scipy.misc.imresize(screen[:,:,0], [84, 84, 1], interp = 'nearest')
        c = scipy.misc.imresize(screen[:,:,1], [84, 84, 1], interp = 'nearest')
        d = scipy.misc.imresize(screen[:,:,2], [84, 84, 1], interp = 'nearest')
        a = np.stack([b,c,d], axis=2)
        return a

    def step(self, action):
        r, screen, valid_actions, course_reached, future_collision = self.moveChar(action)
        state = self.renderEnv(screen)
        return state, r, screen, valid_actions, course_reached, future_collision


# In[4]:

env = gameEnv()


# In[5]:

class Qnetwork():
    def __init__(self,h_size):
        #The network recieves a frame from the game, flattened into an array.
        #It then resizes it and processes it through four convolutional layers.
        self.scalarInput =  tf.placeholder(shape=[None,21168],dtype=tf.float32)
        self.imageIn = tf.reshape(self.scalarInput,shape=[-1,84,84,3])
        self.conv1 = slim.conv2d(             inputs=self.imageIn,num_outputs=32,kernel_size=[8,8],stride=[4,4],padding='VALID', biases_initializer=None)
        self.conv2 = slim.conv2d(             inputs=self.conv1,num_outputs=64,kernel_size=[4,4],stride=[2,2],padding='VALID', biases_initializer=None)
        self.conv3 = slim.conv2d(             inputs=self.conv2,num_outputs=64,kernel_size=[3,3],stride=[1,1],padding='VALID', biases_initializer=None)
        self.conv4 = slim.conv2d(             inputs=self.conv3,num_outputs=h_size,kernel_size=[7,7],stride=[1,1],padding='VALID', biases_initializer=None)

        #We take the output from the final convolutional layer and split it into separate advantage and value streams.
        self.streamAC,self.streamVC = tf.split(self.conv4,2,3)
        self.streamA = slim.flatten(self.streamAC)
        self.streamV = slim.flatten(self.streamVC)
        xavier_init = tf.contrib.layers.xavier_initializer()
        self.AW = tf.Variable(xavier_init([h_size//2,env.actions]))
        self.VW = tf.Variable(xavier_init([h_size//2,1]))
        self.Advantage = tf.matmul(self.streamA,self.AW)
        self.Value = tf.matmul(self.streamV,self.VW)

        #Then combine them together to get our final Q-values.
        self.Qout = self.Value + tf.subtract(self.Advantage,tf.reduce_mean(self.Advantage,axis=1,keep_dims=True))
        self.predict = tf.argmax(self.Qout,1)
        self.extract_value, self.extract_index = tf.nn.top_k(self.Qout, 3, sorted=True)

        #Below we obtain the loss by taking the sum of squares difference between the target and prediction Q values.
        self.targetQ = tf.placeholder(shape=[None],dtype=tf.float32)
        self.actions = tf.placeholder(shape=[None],dtype=tf.int32)
        self.actions_onehot = tf.one_hot(self.actions,env.actions,dtype=tf.float32)

        self.Q = tf.reduce_sum(tf.multiply(self.Qout, self.actions_onehot), axis=1)

        self.td_error = tf.square(self.targetQ - self.Q)
        self.loss = tf.reduce_mean(self.td_error)
        self.trainer = tf.train.AdamOptimizer(learning_rate=0.001)
        self.updateModel = self.trainer.minimize(self.loss)


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


# In[8]:

def updateTargetGraph(tfVars,tau):
    total_vars = len(tfVars)
    op_holder = []
    for idx,var in enumerate(tfVars[0:total_vars//2]):
        op_holder.append(tfVars[idx+total_vars//2].assign((var.value()*tau) + ((1-tau)*tfVars[idx+total_vars//2].value())))
    return op_holder

def updateTarget(op_holder,sess):
    for op in op_holder:
        sess.run(op)


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


# In[10]:

tf.reset_default_graph()
mainQN = Qnetwork(h_size)
targetQN = Qnetwork(h_size)
teacher = mainQN
init = tf.global_variables_initializer()

trainables = tf.trainable_variables()
targetOps = updateTargetGraph(trainables, tau)


# In[ ]:

myBuffer = experience_buffer()

e = startE
epsDrop = (startE - endE)/anneling_steps
beliefDrop = belief/dom_period

rList = []
total_steps = 0

saver = tf.train.Saver(max_to_keep = None)
if not os.path.exists(path):
    os.makedirs(path)


# In[ ]:

with tf.Session() as sess:
    sess.run(init)
    if load_model == True:
        print('Loading Model...')
        # ckpt = tf.train.get_checkpoint_state(load)
        model_name = 'base/model-50.ckpt'
        # saver.restore(sess,ckpt.model_checkpoint_path)
        saver.restore(sess, model_name)
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

            j += 1

            if j == 1:
                a = 14
            else:
                if course_reached:
                    if future_collision:
                        actions_values = sess.run(mainQN.Qout, feed_dict={mainQN.scalarInput:[s]})[0]
                        print('action values = ', actions_values)

                        a = valid_actions[0]
                        for action in valid_actions:
                            # print('action : ', action, 'value = ', actions_values[action])
                            if actions_values[action] > actions_values[a]:
                                a = action
                        previous_action = a
                    else:
                        a = 30
                        previous_action = 30
                else:
                    a = previous_action
            # course goal is from -70 to 70 deg, every 5 deg increment
            if a == 30:
                course_goal = 'base'
            else:
                course_goal = -70 + 5 * a
            # print('At step ', j, 'course goal = ', course_goal, '---reached? ', course_reached)

            state_new, rp, screen_new, valid_actions, course_reached, future_collision = env.step(course_goal)
            if np.abs(rp) >= 50:
                #print(rp)
                flag = True

            s1 = processState(state_new)
            total_steps += 1

            episodeBuffer.add(np.reshape(np.array([s,a,rp,s1,flag]),[1,5]))

            rAll += rp
            s = s1
            screen = screen_new

            if flag == True:
                break
        env.new_game()
        print('episode', i, ', current reward', rAll)
        myBuffer.add(episodeBuffer.buffer)

        rList.append(rAll)
        #Periodically save the model.
        if i > 0 and i % 25 == 0:
            print('episode', i, ', avr r last 25 episodes', np.mean(rList[-25:]),
                  "current e =", e, "total steps =", total_steps)
        if i > 0 and i % 2000 == 0:
            saver.save(sess, path + '/model-' + str(i) + '.ckpt')
            print("Saved Model")
    saver.save(sess,path+'/model-'+str(i)+'.ckpt')


# In[ ]:

rMat = np.resize(np.array(rList), [len(rList)//100, 100])
rMean = np.average(rMat, 1)
plt.plot(rMean)


# In[ ]:

rMat = np.resize(np.array(rList), [len(rList)//100, 100])
rMean = np.average(rMat, 1)
plt.plot(rMean)
plt.ylim(-950, 200)
plt.xlim(-25, 525)


# In[ ]:

with open('test.csv', 'w') as data_dump:
    wr = csv.writer(data_dump)
    wr.writerow(rMean)


# In[ ]:
