# -----------------------------------
# Deep Deterministic Policy Gradient
# Author: Flood Sung
# Date: 2016.5.4
# -----------------------------------
STATE_BASED_ON_IMAGE = False
THREE_HIDDEN_LAYER_ARCHITECTURE = True
import gym
import tensorflow as tf
import numpy as np
from ou_noise import OUNoise
if not STATE_BASED_ON_IMAGE:
    print '--------- Real Value-based Implementation ---------'
    if THREE_HIDDEN_LAYER_ARCHITECTURE:
        from critic_network_3hidden import CriticNetwork 
        from actor_network_bn_3hidden import ActorNetwork
    else:
        from critic_network import CriticNetwork 
        from actor_network_bn import ActorNetwork        
else:
    print '--------- CNN-based Implementation ---------'
    from critic_network_CNN import CriticNetwork 
    from actor_network_CNN import ActorNetwork
from replay_buffer import ReplayBuffer

# Hyper Parameters:

REPLAY_BUFFER_SIZE = 1000000
REPLAY_START_SIZE = 10000
BATCH_SIZE = 64
#BATCH_SIZE = 16 //In the case of Image inputs
GAMMA = 0.99


class DDPG:
    """docstring for DDPG"""
    def __init__(self, env_info_list, configs_path):
        self.name = 'DDPG' # name for uploading results
        self.configs_path = configs_path
        # Randomly initialize actor network and critic network
        # with both their target networks
        self.average_q_value_critic = 0.0
        
        if not STATE_BASED_ON_IMAGE:
            self.state_dim = env_info_list[0]
            BATCH_SIZE = 64
        else:
            self.state_dim = env_info_list[1]
            BATCH_SIZE = 16
        print 'BATCH_SIZE: ', BATCH_SIZE
        self.action_dim = env_info_list[4]
        self.action_min = env_info_list[5]
        self.action_max = env_info_list[6]

        self.sess = tf.InteractiveSession()

        self.actor_network = ActorNetwork(self.sess, self.state_dim, self.action_dim, self.action_min, self.action_max, configs_path)
        self.critic_network = CriticNetwork(self.sess, self.state_dim, self.action_dim, configs_path)
        
        # initialize replay buffer
        self.replay_buffer = ReplayBuffer(REPLAY_BUFFER_SIZE)

        # Initialize a random process the Ornstein-Uhlenbeck process for action exploration
        self.exploration_noise = OUNoise(self.action_dim)        

    def train(self):
        #print "train step",self.time_step
        # Sample a random minibatch of N transitions from replay buffer
        minibatch = self.replay_buffer.get_batch(BATCH_SIZE)
        state_batch = np.asarray([data[0] for data in minibatch])
        action_batch = np.asarray([data[1] for data in minibatch])
        reward_batch = np.asarray([data[2] for data in minibatch])
        next_state_batch = np.asarray([data[3] for data in minibatch])
        done_batch = np.asarray([data[4] for data in minibatch])

        # for action_dim = 1
        action_batch = np.resize(action_batch,[BATCH_SIZE, self.action_dim])

        # Calculate y_batch
        
        next_action_batch = self.actor_network.target_actions(next_state_batch)
        q_value_batch = self.critic_network.target_q(next_state_batch, next_action_batch)
        q_value_critic_batch = self.critic_network.q_value(next_state_batch, next_action_batch)
        self.average_q_value_critic = np.mean(q_value_critic_batch)
        #print 'q_value_critic (average): ', average_q_value_critic
        #print 'q_value_critic (type): ', type(q_value_critic_batch)
        y_batch = []
        for i in range(len(minibatch)): 
            if done_batch[i]:
                y_batch.append(reward_batch[i])
            else :
                y_batch.append(reward_batch[i] + GAMMA * q_value_batch[i])
        y_batch = np.resize(y_batch,[BATCH_SIZE,1])
        # Update critic by minimizing the loss L
        self.critic_network.train(y_batch, state_batch, action_batch)

        # Update the actor policy using the sampled gradient:
        action_batch_for_gradients = self.actor_network.actions(state_batch)
        q_gradient_batch = self.critic_network.gradients(state_batch,action_batch_for_gradients)

        self.actor_network.train(q_gradient_batch, state_batch)

        # Update the target networks
        self.actor_network.update_target()
        self.critic_network.update_target()

    def noise_action(self,state):
        # Select action a_t according to the current policy and exploration noise
        action = self.actor_network.action(state)
        action_noise = action + self.exploration_noise.noise()
        #~ print 'action: ', action
        #~ print 'noise: ', self.exploration_noise.noise()

        # Saturate noise
        for i in range(len(action_noise)):
            if action_noise[i] > self.action_max[i]:
                action_noise[i] = self.action_max[i]
            if action_noise[i] < self.action_min[i]:
                action_noise[i] = self.action_min[i]

        return action_noise

    def action(self,state):
        action = self.actor_network.action(state)
        return action

    def perceive(self, state, action, reward, next_state, done):
        # Store transition (s_t,a_t,r_t,s_{t+1}) in replay buffer
        self.replay_buffer.add(state, action, reward, next_state, done)

        # Store transitions to replay start size then start training
        if self.replay_buffer.count() >  REPLAY_START_SIZE:
            self.train()

        #if self.time_step % 10000 == 0:
            #self.actor_network.save_network(self.time_step)
            #self.critic_network.save_network(self.time_step)

        # Re-iniitialize the random process when an episode ends
        if done:
            self.exploration_noise.reset()










