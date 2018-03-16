import filter_env
import rospy
from rl_agent_environment_communication.srv import *
import cv2
from cv_bridge import CvBridge
import gym
import numpy as np

ENV_NAME = 'LunarLanderContinuous-v2'
#ENV_NAME = 'Pendulum-v0'

DEBUG_SERVICES_MODE = False

env = filter_env.makeFilteredEnv(gym.make(ENV_NAME))
#env = gym.wrappers.Monitor(env, 'experiments/' + ENV_NAME,force=True)
state = env.reset()


def handle_environment_reset(req):
	print 'Reseting Env...'
	state = env.reset()
	
	resp = ResetEnvSrvResponse()
	resp.state = state
	return resp
	
	
def handle_environment_render(req):
	print 'Rendering Env...'
	image_state = env.render(mode='rgb_array')
	print 'ENV RENDERED!'
	
	#~ size = 400, 600, 3
	#~ image_state = np.zeros(size, dtype=np.uint8)
	#~ print 'image_state (type): ', type(image_state)
	#~ cv2.imshow('image_state', image_state)
	#~ cv2.waitKey(1)
	
	print 'Creating CvBridge object...'
	cv_bridge_obj = CvBridge()
	img_msg = cv_bridge_obj.cv2_to_imgmsg(image_state, "bgr8")
	
	print 'Sending response...'
	resp = RenderEnvSrvResponse()
	resp.img = img_msg
	return resp	


def handle_environment_step(req):
	#env.render()
	action_req = np.array(req.action)
		
	if DEBUG_SERVICES_MODE:
		print '++++++++++ Requested Action INFO ++++++++++'
		print 'action (type): ', type(action_req)
		print 'action (shape): ', action_req.shape
		print 'action (length): ', len(action_req)
		print 'action (dim): ', action_req.ndim
		print 'action (values): ', action_req

	next_state, reward, done, _ = env.step(action_req)
	
	
	size = 5, 5, 3
	img = np.zeros(size, dtype=np.uint8)
	cv_bridge_obj = CvBridge()
	img_msg = cv_bridge_obj.cv2_to_imgmsg(img, "bgr8")
	
	resp = AgentSrvResponse()
	resp.reward = reward
	resp.obs_real = np.array(next_state)
	resp.terminal_state = done
	resp.img = img_msg
	return resp
	
	
def handle_environment_dimensionality(req):
	state_dim = env.observation_space.shape[0]
	action_dim = env.action_space.shape[0]
	state_dim_low = env.observation_space.low
	state_dim_high = env.observation_space.high
	action_dim_low = env.action_space.low
	action_dim_high = env.action_space.high
	
	if DEBUG_SERVICES_MODE:
		print '**** state_dim: ', state_dim
		print '**** action_dim: ', action_dim
		print '**** state_dim_low: ', state_dim_low
		print '**** state_dim_low (type): ', type(state_dim_low)
		print '**** state_dim_high: ', state_dim_high
		print '**** action_dim_low: ', action_dim_low
		print '**** action_dim_low (type): ', type(action_dim_low)
		print '**** action_dim_high: ', action_dim_high
	
	
	resp = EnvDimensionalitySrvResponse()
	resp.state_dim_lowdim = state_dim
	resp.state_dim_img = np.array([5, 5, 3], dtype=np.int)
	resp.state_min = state_dim_low
	resp.state_max = state_dim_high
	resp.action_dim = action_dim
	resp.action_min = action_dim_low
	resp.action_max = action_dim_high
	resp.num_iterations = env.spec.timestep_limit
	return resp
		


def environment_server():
    rospy.init_node('environment_step_server')
    s1 = rospy.Service('environment_step', AgentSrv, handle_environment_step)
    s2 = rospy.Service('environment_dimensionality', EnvDimensionalitySrv, handle_environment_dimensionality)
    s3 = rospy.Service('environment_reset', ResetEnvSrv, handle_environment_reset)
    s4 = rospy.Service('environment_render', RenderEnvSrv, handle_environment_render)
    print "Ready to step the Environment..."
    rospy.spin()
     
    
    
def main():
    environment_server()
    


if __name__ == '__main__':
    main()

