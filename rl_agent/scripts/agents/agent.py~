#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from rl_agent.srv import *

import os,sys,inspect
sys.path.insert(0,os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))))))

import cv2
from cv_bridge import CvBridge

import matplotlib.pyplot as plt
import time
import numpy as np

# Classic ddpg import
from scripts.agents.ddpg_agent import *
from scripts.agents.ddpg_agent.ddpg import DDPG
from scripts.agents.ddpg_agent.trained_actor_network_3hidden import TrainedActorNetwork  #For testing 3 hidden-layered Actor Network

from scripts.misc import filter_env

rl_boost_python_path = rospy.get_param('/agent/rl_boost_python_path')
rl_python_distpackages_path = rospy.get_param('/agent/rl_python_distpackages_path')
sys.path.insert(1, rl_boost_python_path)
sys.path.insert(2, rl_python_distpackages_path)

from librl_shm_python import RlSharedMemory

import gc
gc.enable()

DEBUG_SERVICES_MODE = False
DEBUG_MODE = False
TEST_MODE = True
ENABLE_ITERATION = False
STATE_BASED_ON_IMAGE = False

configs_path = rospy.get_param('/agent/configs_path')
exp_logger_path = rospy.get_param('/agent/exp_logger_path')
actor_networks_path = rospy.get_param('/agent/actor_networks_path')
agent_type = rospy.get_param('/agent/agent_type')
EPISODES = rospy.get_param('/agent/num_episodes')
NUM_EPISODES_BETWEEN_LOGS = rospy.get_param('/agent/num_episodes_between_logs')
TEST = rospy.get_param('/agent/num_tests')
FREQUENCY = rospy.get_param('/agent/frecuency')

print 'NUM_EPISODES: ', EPISODES
print 'NUM_EPISODES_LOGS', NUM_EPISODES_BETWEEN_LOGS
print 'NUM_TESTS: ', TEST
print 'FREQUENCY: ', FREQUENCY
print 'AGENT TYPE: ', agent_type

def experiment_record_client(num_episode, path_name):
    rospy.wait_for_service('experiment_record_srv')
    try:
        experiment_record_srv = rospy.ServiceProxy('experiment_record_srv', RecordExperimentSrv)

        request = RecordExperimentSrvRequest()
        request.episode_num = num_episode
        request.path_dir = path_name
        request.record_with_camera = True

        experiment_record_srv(request)

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def environment_render_client():
    rospy.wait_for_service('environment_render')
    try:
        environment_render = rospy.ServiceProxy('environment_render', RenderEnvSrv)
        resp = environment_render()

        image_response = resp.img

        cv_bridge_obj = CvBridge()
        image_np = cv_bridge_obj.imgmsg_to_cv2(resp.img, "rgb8")
        print '------- Response (env render) -------'
        print 'resp.img (type): ', type(image_np)

        return image_np
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def environment_reset_client():
    rospy.wait_for_service('rl_env_reset_srv')
    try:
        environment_reset = rospy.ServiceProxy('rl_env_reset_srv', ResetEnvSrv)
        resp = environment_reset()

        print '------- Response (env reset) -------'
        print 'resp.state: ', resp.state

        image_response = resp.img_state
        if(len(image_response) !=0):
            cv_bridge_obj = CvBridge()
            image_ini = cv_bridge_obj.imgmsg_to_cv2(image_response[0], "mono8")
            images_array = np.empty((image_ini.shape[0], image_ini.shape[1], len(image_response)))
            print 'len(image_response): ', len(image_response)
            for i in range(len(image_response)):
                image_np = cv_bridge_obj.imgmsg_to_cv2(image_response[i], "mono8")
                image_np_norm = np.multiply(image_np, 1.0 / 255.0)
                images_array[:,:,i] = np.reshape(image_np_norm, (image_np_norm.shape[0], image_np_norm.shape[1]))
            print 'images_array (shape): ', images_array.shape
            return resp.state, images_array
        else:
            return resp.state, image_response
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def environment_step_client(action):
    #print 'waiting for server (step_client)...'
    rospy.wait_for_service('rl_env_step_srv')
    try:
        environment_step = rospy.ServiceProxy('rl_env_step_srv', AgentSrv)
        resp = environment_step(action)

        if DEBUG_SERVICES_MODE:
            print '------- Response (env step) -------'
            print 'resp.reward: ', resp.reward
            print 'resp.state: ', resp.obs_real
            print 'resp.terminal_state: ', resp.terminal_state
            print 'resp.img (type): ', type(image_np)

        image_response = resp.img
        if(len(image_response) !=0):
            cv_bridge_obj = CvBridge()
            image_ini = cv_bridge_obj.imgmsg_to_cv2(image_response[0], "mono8")
            images_array = np.empty((image_ini.shape[0], image_ini.shape[1], len(image_response)))
            for i in range(len(image_response)):
                image_np = cv_bridge_obj.imgmsg_to_cv2(image_response[i], "mono8")
                image_np_norm = np.multiply(image_np, 1.0 / 255.0)
                images_array[:,:,i] = np.reshape(image_np_norm, (image_np_norm.shape[0], image_np_norm.shape[1]))
            return resp.obs_real, resp.reward, resp.terminal_state, images_array
        else:
            return resp.obs_real, resp.reward, resp.terminal_state, image_response
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def environment_dimensionality_client():
    print 'waiting for server (dimensionality_client)...'
    rospy.wait_for_service('rl_env_dimensionality_srv')
    try:
        environment_dimensionality = rospy.ServiceProxy('rl_env_dimensionality_srv', EnvDimensionalitySrv)
        resp = environment_dimensionality()

        if DEBUG_SERVICES_MODE:
            print '------- Response (env dimensionality) -------'
            print 'resp.state_dim_lowdim: ', resp.state_dim_lowdim
            print 'resp.state_dim_img: ', resp.state_dim_img
            print 'resp.state_min: ', resp.state_min
            print 'resp.state_max: ', resp.state_max
            print 'resp.action_dim: ', resp.action_dim
            print 'resp.action_min: ', resp.action_min
            print 'resp.action_max: ', resp.action_max
            print 'resp.num_iterations: ', resp.num_iterations

        return resp.state_dim_lowdim, resp.state_dim_img, resp.state_min, resp.state_max, resp.action_dim, resp.action_min, resp.action_max, resp.num_iterations
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def Init():
    state_dim_lowdim, state_dim_img, state_min, state_max, action_dim, action_min, action_max, num_iterations = environment_dimensionality_client()

    return [state_dim_lowdim, state_dim_img, state_min, state_max, action_dim, action_min, action_max, num_iterations]

def main():
    env_info_list = Init()
    print 'Env Info List:'
    print env_info_list

    if not TEST_MODE:
        print "AGENT_ERROR: This code is not meant for training"
    else:
        if agent_type == 'ddpg':
            agent = TrainedActorNetwork(env_info_list, actor_networks_path)
        else:
            print "AGENT_ERROR: Agent type not recognized"

    if not ENABLE_ITERATION:
        r = rospy.Rate(FREQUENCY)


    # Initialize shared memory object
    rl_shm = RlSharedMemory()

    for episode in xrange(EPISODES):
        if ENABLE_ITERATION:
            rl_shm.CallAction()

        reward = 0

        state_resp, img_resp = environment_reset_client()
        state = np.array(state_resp)

        print "Test episode: ", episode

        ############## Test ##############
        if TEST_MODE:
            total_reward = 0
            while(TEST_MODE):

                accum_reward = 0

                if ENABLE_ITERATION:
                    rl_shm.CallAction()

                #state_resp, img_resp = environment_reset_client()
                #state = np.array(state_resp)

		if agent_type == 'ddpg':
		    action = agent.action(state)  # direct action for test
		else:
		    print "AGENT_ERROR: Agent type not recognized"

		if ENABLE_ITERATION:
		    rl_shm.CallAction()
        	state_resp, reward, done, image_state = environment_step_client(action)
		state = np.array(state_resp)

		if done:
		    break

		# Check time
		if not ENABLE_ITERATION:
		    r.sleep()


if __name__ == '__main__':
    rospy.init_node('environment_step_client')
    main()




