
import tensorflow as tf 
import numpy as np
import math
from configobj import ConfigObj

#~ LAYER1_SIZE = 300
#~ LAYER2_SIZE = 200
#~ LEARNING_RATE = 1e-3
#~ TAU = 0.001
#~ L2 = 0.01

class CriticNetwork:
	"""docstring for CriticNetwork"""
	def __init__(self, sess, state_dim, action_dim, configs_path):
		self.time_step = 0
		self.sess = sess
		config = ConfigObj(configs_path)
		Critic = config['DDPG']['Critic']
		self.layer1_size = int(Critic['LAYER1_SIZE'])
		self.layer2_size = int(Critic['LAYER2_SIZE'])
		self.layer3_size = int(Critic['LAYER3_SIZE'])
		self.learning_rate = float(Critic['LEARNING_RATE'])
		self.tau = float(Critic['TAU'])
		self.L2 = float(Critic['L2'])

		print 'LAYER1_SIZE (Critic): ', self.layer1_size
		print 'LAYER2_SIZE (Critic): ', self.layer2_size
		print 'LAYER3_SIZE (Critic): ', self.layer3_size
		print 'LEARNING_RATE (Critic): ', self.learning_rate
		print 'TAU (Critic): ', self.tau
		print 'L2 (Critic): ', self.L2
		# create q network
		self.state_input,\
		self.action_input,\
		self.q_value_output,\
		self.net = self.create_q_network(state_dim,action_dim)

		# create target q network (the same structure with q network)
		self.target_state_input,\
		self.target_action_input,\
		self.target_q_value_output,\
		self.target_update = self.create_target_q_network(state_dim,action_dim,self.net)

		self.create_training_method()

		# initialization 
		# self.sess.run(tf.initialize_all_variables())
		self.sess.run(tf.global_variables_initializer())
			
		self.update_target()

	def create_training_method(self):
		# Define training optimizer
		self.y_input = tf.placeholder("float",[None,1])
		weight_decay = tf.add_n([self.L2 * tf.nn.l2_loss(var) for var in self.net])
		self.cost = tf.reduce_mean(tf.square(self.y_input - self.q_value_output)) + weight_decay
		self.optimizer = tf.train.AdamOptimizer(self.learning_rate).minimize(self.cost)
		#self.optimizer = tf.train.AdadeltaOptimizer().minimize(self.cost)
		self.action_gradients = tf.gradients(self.q_value_output,self.action_input)

	def create_q_network(self, state_dim, action_dim):

		state_input = tf.placeholder("float",[None,state_dim])
		action_input = tf.placeholder("float",[None,action_dim])

		W1 = self.variable([state_dim,self.layer1_size],state_dim)
		b1 = self.variable([self.layer1_size],state_dim)
		W2 = self.variable([self.layer1_size,self.layer2_size],self.layer1_size+action_dim)
		W2_action = self.variable([action_dim,self.layer2_size],self.layer1_size+action_dim)
		b2 = self.variable([self.layer2_size],self.layer1_size+action_dim)
		W3 = self.variable([self.layer2_size,self.layer3_size],self.layer2_size)
		b3 = self.variable([self.layer3_size],self.layer2_size)
		W4 = tf.Variable(tf.random_uniform([self.layer3_size, 1],-3e-3,3e-3))
		b4 = tf.Variable(tf.random_uniform([1],-3e-3,3e-3))        

		layer1 = tf.nn.relu(tf.matmul(state_input,W1) + b1)
		layer2 = tf.nn.relu(tf.matmul(layer1,W2) + tf.matmul(action_input,W2_action) + b2)
		layer3 = tf.nn.relu(tf.matmul(layer2,W3) + b3)
		q_value_output = tf.identity(tf.matmul(layer3,W4) + b4)

		return state_input,action_input,q_value_output,[W1,b1,W2,W2_action,b2,W3,b3,W4,b4]

	def create_target_q_network(self,state_dim,action_dim,net):
		state_input = tf.placeholder("float",[None,state_dim])
		action_input = tf.placeholder("float",[None,action_dim])

		ema = tf.train.ExponentialMovingAverage(decay=1-self.tau)
		target_update = ema.apply(net)
		target_net = [ema.average(x) for x in net]

		layer1 = tf.nn.relu(tf.matmul(state_input,target_net[0]) + target_net[1])
		layer2 = tf.nn.relu(tf.matmul(layer1,target_net[2]) + tf.matmul(action_input,target_net[3]) + target_net[4])
		layer3 = tf.nn.relu(tf.matmul(layer2,target_net[5]) + target_net[6])        
		q_value_output = tf.identity(tf.matmul(layer3,target_net[7]) + target_net[8])

		return state_input,action_input,q_value_output,target_update

	def update_target(self):
		self.sess.run(self.target_update)

	def train(self,y_batch,state_batch,action_batch):
		self.time_step += 1
		self.sess.run(self.optimizer,feed_dict={
			self.y_input:y_batch,
			self.state_input:state_batch,
			self.action_input:action_batch
			})

	def gradients(self,state_batch,action_batch):
		return self.sess.run(self.action_gradients,feed_dict={
			self.state_input:state_batch,
			self.action_input:action_batch
			})[0]

	def target_q(self,state_batch,action_batch):
		return self.sess.run(self.target_q_value_output,feed_dict={
			self.target_state_input:state_batch,
			self.target_action_input:action_batch
			})

	def q_value(self,state_batch,action_batch):
		return self.sess.run(self.q_value_output,feed_dict={
			self.state_input:state_batch,
			self.action_input:action_batch})

	# f fan-in size
	def variable(self,shape,f):
		return tf.Variable(tf.random_uniform(shape,-1/math.sqrt(f),1/math.sqrt(f)))
'''
	def load_network(self):
		self.saver = tf.train.Saver()
		checkpoint = tf.train.get_checkpoint_state("saved_critic_networks")
		if checkpoint and checkpoint.model_checkpoint_path:
			self.saver.restore(self.sess, checkpoint.model_checkpoint_path)
			print "Successfully loaded:", checkpoint.model_checkpoint_path
		else:
			print "Could not find old network weights"

	def save_network(self,time_step):
		print 'save critic-network...',time_step
		self.saver.save(self.sess, 'saved_critic_networks/' + 'critic-network', global_step = time_step)
'''
		
