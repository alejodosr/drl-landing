#!/usr/bin/env python

import sys

import rospy
from sensor_msgs.msg import Image
from rl_agent.srv import *
import std_srvs
from std_srvs.srv import Empty
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from trained_actor_network import *
from droneMsgsROS.msg import droneSpeeds
import csv

class AgentRosModule:
    def __init__(self):
        self.ReadParameters()
        self.uav_speed_ref_pub_ = rospy.Publisher('/drone7/droneSpeedsRefs', droneSpeeds, queue_size=10)
        self.uav_state_actions_pub_ = rospy.Publisher('/drone7/droneRlStateActions', Float32MultiArray, queue_size=10)
        self.rl_env_state_subs_ = rospy.Subscriber('rl_environment/state', Float32MultiArray, self.EnvironmentStateCallback)
        
        self.env_info_list = self.EnvironmentDimensionalityClient()
        print 'self.env_info_list: ', self.env_info_list
        self.agent = TrainedActorNetwork(self.env_info_list, self.actor_networks_path)
        self.state_dim = 4
        with open(self.record_results_path, 'w') as fp:
            writer = csv.writer(fp, delimiter=',')
            if self.state_dim == 4:
                data = ['state_x', 'state_y', 'state_vx', 'state_vy', 'action_x', 'action_y', 'reward']
            elif self.state_dim == 2:
                data = ['state_x', 'state_y', 'action_x', 'action_y']
            writer.writerow(data)
        
    def ReadParameters(self):
        self.exp_logger_path = rospy.get_param('/gym_ddpg_agent/exp_logger_path')
        self.actor_networks_path = rospy.get_param('/gym_ddpg_agent/actor_networks_path')
        self.record_results_path = rospy.get_param('/gym_ddpg_agent/record_resutls_path')
        self.num_espisodes = rospy.get_param('/gym_ddpg_agent/num_episodes')
        self.num_episodes_between_logs = rospy.get_param('/gym_ddpg_agent/num_episodes_between_logs')
        self.num_tests = rospy.get_param('/gym_ddpg_agent/num_tests')
        self.frequency = rospy.get_param('/gym_ddpg_agent/frecuency')


    def EnvironmentDimensionalityClient(self):
        rospy.wait_for_service('rl_env_dimensionality_srv')
        try:
            print 'Executing service proxy...'
            environment_dimensionality = rospy.ServiceProxy('rl_env_dimensionality_srv', EnvDimensionalitySrv)
            resp = environment_dimensionality()
            return [resp.state_dim_lowdim, resp.state_dim_img, resp.state_min, resp.state_max, resp.action_dim, resp.action_min, resp.action_max, resp.num_iterations]
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
    def EnvironmentStepClient(self, action):
        #print 'waiting for server (step_client)...'
        rospy.wait_for_service('rl_env_step_srv')
        try:
            environment_step = rospy.ServiceProxy('rl_env_step_srv', AgentSrv)
            resp = environment_step(action)
            return resp.obs_real, resp.reward, resp.terminal_state, resp.img
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
        
    def EnvironmentStateCallback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        
        state = data.data
        #print 'received state: ', state
        action = self.agent.action(state)
        
        if not np.isfinite(action).all():
            action = np.zeros(len(action))
            
        action_msg = droneSpeeds()
        action_msg.dx = action[0]
        action_msg.dy = action[1]
        action_msg.dz = 0.0
        self.uav_speed_ref_pub_.publish(action_msg)

        state_array = np.asarray(state)
        state_actions_array = np.concatenate((action, state_array))
        rl_state_ations_msg = Float32MultiArray()
        rl_state_ations_msg.layout.dim.append(MultiArrayDimension())
        rl_state_ations_msg.layout.dim[0].size = len(state_actions_array)
        rl_state_ations_msg.layout.dim[0].stride = 1
        rl_state_ations_msg.layout.dim[0].label = "x"
        rl_state_ations_msg.data = state_actions_array
        self.uav_state_actions_pub_.publish(rl_state_ations_msg)
        
        
        next_state_resp, reward, done, img = self.EnvironmentStepClient(action)
        #print 'reward: ', reward;
        
        with open(self.record_results_path, 'a') as f:
            writer = csv.writer(f)
            if len(state) == 4:
                writer.writerow([str(state[0]), str(state[1]), str(state[2]), str(state[3]), str(action[0]), str(action[1]), str(reward)])
            elif len(state) == 2:
                writer.writerow([str(state[0]), str(state[1]), str(action[0]), str(action[1])])
