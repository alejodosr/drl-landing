import filter_env
import rospy
from rl_agent_environment_communication.srv import *
import cv2
from cv_bridge import CvBridge
import gym
from gym import wrappers
import numpy as np

ENV_NAME = 'LunarLanderContinuous-v2'
#ENV_NAME = 'Pendulum-v0'

DEBUG_SERVICES_MODE = False




class GymDDPGServer:
	def __init__(self, environment, state):
		self.env = environment
		self.state = state
		#self.env = filter_env.makeFilteredEnv(gym.make(ENV_NAME))
		#self.env = gym.wrappers.Monitor(self.env, 'experiments/' + ENV_NAME,force=True)
		#self.state = self.env.reset()
		
	def Open(self):
		rospy.init_node('environment_step_server')
		self.s1 = rospy.Service('environment_step', AgentSrv, self.handle_environment_step)
		self.s2 = rospy.Service('environment_dimensionality', EnvDimensionalitySrv, self.handle_environment_dimensionality)
		self.s3 = rospy.Service('environment_reset', ResetEnvSrv, self.handle_environment_reset)
		self.s4 = rospy.Service('environment_render', RenderEnvSrv, self.handle_environment_render)
		print "Server Services Ready..."
		
	def handle_environment_reset(self, req):
		print 'Reseting Env...'
		self.state = self.env.reset()
		
		resp = ResetEnvSrvResponse()
		resp.state = self.state
		return resp
	
	
	def handle_environment_render(self, req):
		print 'Rendering Env...'
		#~ image_state = self.env.render(mode='rgb_array')
		#~ cv2.imshow('image_state', image_state)
		#~ cv2.waitKey()
		#~ print 'ENV RENDERED!'
		
		size = 400, 600, 3
		image_state = np.zeros(size, dtype=np.uint8)
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


	def handle_environment_step(self, req):
		action_req = np.array(req.action)
			
		if DEBUG_SERVICES_MODE:
			print '++++++++++ Requested Action INFO ++++++++++'
			print 'action (type): ', type(action_req)
			print 'action (shape): ', action_req.shape
			print 'action (length): ', len(action_req)
			print 'action (dim): ', action_req.ndim
			print 'action (values): ', action_req

		next_state, reward, done, _ = self.env.step(action_req)
		
		
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
		
		
	def handle_environment_dimensionality(self, req):
		state_dim = self.env.observation_space.shape[0]
		action_dim = self.env.action_space.shape[0]
		state_dim_low = self.env.observation_space.low
		state_dim_high = self.env.observation_space.high
		action_dim_low = self.env.action_space.low
		action_dim_high = self.env.action_space.high
		
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
		resp.num_iterations = self.env.spec.timestep_limit
		return resp
		
    
    
def main():
	env = filter_env.makeFilteredEnv(gym.make(ENV_NAME))
	#env = gym.wrappers.Monitor(env, 'experiments/' + ENV_NAME,force=True)
	state = env.reset()
	
	ddpg_server = GymDDPGServer(env, state)
	ddpg_server.Open()

	rospy.spin()


if __name__ == '__main__':
    main()

