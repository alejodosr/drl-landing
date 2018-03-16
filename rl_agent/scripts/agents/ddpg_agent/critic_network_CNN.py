
import tensorflow as tf 
import numpy as np
import math
from configobj import ConfigObj

#~ LAYER1_SIZE = 300
#~ LAYER2_SIZE = 200
#~ LEARNING_RATE = 1e-3
#~ TAU = 0.001
#~ L2 = 0.01

num_feature_maps_c1 = 16
num_feature_maps_c2 = 32
NUM_HIDDEN_UNITS_fc1 = 256

class CriticNetwork:
	"""docstring for CriticNetwork"""
	def __init__(self, sess, state_dim, action_dim, configs_path):
		self.time_step = 0
		self.sess = sess
		config = ConfigObj(configs_path)
		Critic = config['DDPG']['Critic']
		self.layer1_size = int(Critic['LAYER1_SIZE'])
		self.layer2_size = int(Critic['LAYER2_SIZE'])
		self.learning_rate = float(Critic['LEARNING_RATE'])
		self.tau = float(Critic['TAU'])
		self.L2 = float(Critic['L2'])

		print 'LAYER1_SIZE (Critic): ', self.layer1_size
		print 'LAYER2_SIZE (Critic): ', self.layer2_size
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
        
		#~ ## CNN Implementation
		state_input = tf.placeholder("float", shape=[None, state_dim[0], state_dim[1], state_dim[2]]) #[None, width, height, num_channels]
		action_input = tf.placeholder("float",[None, action_dim])
		is_training = tf.placeholder(tf.bool)
		x_image = tf.reshape(state_input, [-1, state_dim[0], state_dim[1], state_dim[2]])
		input_dim_fc1 = state_dim[0] // 4 * state_dim[1] // 4 * num_feature_maps_c2 #For a 2-Pooling layers of 2x2

		W_conv1 = tf.Variable(tf.truncated_normal([5, 5, state_dim[2], num_feature_maps_c1], stddev=0.1))
		b_conv1 =  tf.Variable(tf.constant(0.1, shape=[num_feature_maps_c1]))
		W_conv2 = tf.Variable(tf.truncated_normal([5, 5, num_feature_maps_c1, num_feature_maps_c2], stddev=0.1))
		b_conv2 = tf.Variable(tf.constant(0.1, shape=[num_feature_maps_c2]))
		W_fc1 = self.variable([input_dim_fc1, NUM_HIDDEN_UNITS_fc1], input_dim_fc1+action_dim)
 		W_fc1_action = self.variable([action_dim, NUM_HIDDEN_UNITS_fc1], input_dim_fc1+action_dim)
		b_fc1 = self.variable([NUM_HIDDEN_UNITS_fc1], input_dim_fc1+action_dim)
		W_fc2 = tf.Variable(tf.random_uniform([NUM_HIDDEN_UNITS_fc1, 1], 3e-4, 3e-4))
		b_fc2 = tf.Variable(tf.random_uniform([1], 3e-4, 3e-4))                

		h_conv1 = tf.nn.relu(tf.nn.conv2d(x_image, W_conv1, strides=[1, 1, 1, 1], padding='SAME') + b_conv1)
		h_pool1 = tf.nn.max_pool(h_conv1, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding='SAME')
		h_conv2 = tf.nn.relu(tf.nn.conv2d(h_pool1, W_conv2, strides=[1, 1, 1, 1], padding='SAME') + b_conv2)
		h_pool2 = tf.nn.max_pool(h_conv2, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding='SAME')
		h_pool2_flat = tf.reshape(h_pool2, [-1, input_dim_fc1])
		h_fc1 = tf.nn.relu(tf.matmul(h_pool2_flat, W_fc1) + tf.matmul(action_input, W_fc1_action) + b_fc1)
		q_value_output = tf.identity(tf.matmul(h_fc1, W_fc2) + b_fc2) 

		#~ state_input = tf.placeholder("float",[None,state_dim])
		#~ action_input = tf.placeholder("float",[None,action_dim])
		#~ W1 = self.variable([state_dim,self.layer1_size],state_dim)
		#~ b1 = self.variable([self.layer1_size],state_dim)
		#~ W2 = self.variable([self.layer1_size,self.layer2_size],self.layer1_size+action_dim)
		#~ W2_action = self.variable([action_dim,self.layer2_size],self.layer1_size+action_dim)
		#~ b2 = self.variable([self.layer2_size],self.layer1_size+action_dim)
		#~ W3 = tf.Variable(tf.random_uniform([self.layer2_size,1],-3e-3,3e-3))
		#~ b3 = tf.Variable(tf.random_uniform([1],-3e-3,3e-3))
		#~ layer1 = tf.nn.relu(tf.matmul(state_input,W1) + b1)
		#~ layer2 = tf.nn.relu(tf.matmul(layer1,W2) + tf.matmul(action_input,W2_action) + b2)
		#~ q_value_output = tf.identity(tf.matmul(layer2,W3) + b3)
		#~ return state_input,action_input,q_value_output,[W1,b1,W2,W2_action,b2,W3,b3]
        
		return state_input,action_input,q_value_output, [W_conv1, b_conv1, W_conv2, b_conv2, W_fc1, W_fc1_action, b_fc1, W_fc2, b_fc2]

	def create_target_q_network(self, state_dim, action_dim,net):
		state_input = tf.placeholder("float", shape=[None, state_dim[0], state_dim[1], state_dim[2]]) #[None, width, height, num_channels]
		action_input = tf.placeholder("float", [None, action_dim])

		ema = tf.train.ExponentialMovingAverage(decay=1-self.tau)
		target_update = ema.apply(net)
		target_net = [ema.average(x) for x in net]

		state_input_image = tf.reshape(state_input, [-1, state_dim[0], state_dim[1], state_dim[2]])
		input_dim_fc1 = state_dim[0] // 4 * state_dim[1] // 4 * num_feature_maps_c2 #For a 2-Pooling layers of 2x2     
		h_conv1 = tf.nn.relu(tf.nn.conv2d(state_input_image, target_net[0], strides=[1, 1, 1, 1], padding='SAME') + target_net[1])
		h_pool1 = tf.nn.max_pool(h_conv1, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding='SAME')
		h_conv2 = tf.nn.relu(tf.nn.conv2d(h_pool1, target_net[2], strides=[1, 1, 1, 1], padding='SAME') + target_net[3])
		h_pool2 = tf.nn.max_pool(h_conv2, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding='SAME')
		h_pool2_flat = tf.reshape(h_pool2, [-1, input_dim_fc1])
		h_fc1 = tf.nn.relu(tf.matmul(h_pool2_flat, target_net[4]) + tf.matmul(action_input, target_net[5]) + target_net[6])
		q_value_output = tf.identity(tf.matmul(h_fc1, target_net[7]) + target_net[8])

		#~ layer1 = tf.nn.relu(tf.matmul(state_input,target_net[0]) + target_net[1])
		#~ layer2 = tf.nn.relu(tf.matmul(layer1,target_net[2]) + tf.matmul(action_input,target_net[3]) + target_net[4])
		#~ q_value_output = tf.identity(tf.matmul(layer2,target_net[5]) + target_net[6])

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
		
