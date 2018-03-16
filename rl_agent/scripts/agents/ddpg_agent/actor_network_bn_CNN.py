import tensorflow as tf 
from tensorflow.contrib.layers.python.layers import batch_norm as batch_norm
import numpy as np
import math
from configobj import ConfigObj


# Hyper Parameters
#~ LAYER1_SIZE = 300
#~ LAYER2_SIZE = 200
#~ LEARNING_RATE = 1e-4
#~ TAU = 0.001
#~ BATCH_SIZE = 64



image_width = 64 # width of the image in pixels 
image_height = 64 # height of the image in pixels
flat = image_width * image_height # number of pixels in one image 
image_channels = 1
num_feature_maps_c1 = 16
num_feature_maps_c2 = 32
NUM_HIDDEN_UNITS_fc1 = 256


class ActorNetwork:
	"""docstring for ActorNetwork"""
	def __init__(self, sess, state_dim, action_dim, action_min, action_max, configs_path):
		self.sess = sess
		self.state_dim = state_dim
		self.action_dim = action_dim
		self.action_max = action_max
		self.action_min = action_min
		config = ConfigObj(configs_path)
		Actor = config['DDPG']['Actor']
		self.layer1_size = int(Actor['LAYER1_SIZE'])
		self.layer2_size = int(Actor['LAYER2_SIZE'])
		self.learning_rate = float(Actor['LEARNING_RATE'])
		self.tau = float(Actor['TAU'])
		self.batch_size = int(Actor['BATCH_SIZE'])

		print 'LAYER1_SIZE (Actor): ', self.layer1_size
		print 'LAYER2_SIZE (Actor): ', self.layer2_size
		print 'LEARNING_RATE (Actor): ', self.learning_rate
		print 'TAU (Actor): ', self.tau
		print 'BATCH_SIZE (Actor): ', self.batch_size     
		# create actor network
		self.state_input,self.action_output,self.net,self.is_training = self.create_network(state_dim,action_dim)

		# create target actor network
		self.target_state_input,self.target_action_output,self.target_update,self.target_is_training = self.create_target_network(state_dim,action_dim,self.net)

		# define training rules
		self.create_training_method()

		# self.sess.run(tf.initialize_all_variables())
		self.sess.run(tf.global_variables_initializer())

		self.update_target()
		self.load_network()

	def create_training_method(self):
		self.q_gradient_input = tf.placeholder("float",[None,self.action_dim])
		self.parameters_gradients = tf.gradients(self.action_output,self.net,-self.q_gradient_input)
		self.optimizer = tf.train.AdamOptimizer(self.learning_rate).apply_gradients(zip(self.parameters_gradients,self.net))
		#self.optimizer = tf.train.AdadeltaOptimizer().apply_gradients(zip(self.parameters_gradients,self.net))

	def create_network(self, state_dim, action_dim):
		#~ layer1_size = LAYER1_SIZE
		#~ layer2_size = LAYER2_SIZE

		#~ ## CNN Implementation
		#~ state_input = tf.placeholder("float", shape=[None, state_dim[0], state_dim[1], state_dim[2]]) #[None, width, height, num_channels]
		#~ is_training = tf.placeholder(tf.bool)
		#~ state_input_image = tf.reshape(state_input, [-1, state_dim[0], state_dim[1], state_dim[2]])
		#~ input_dim_fc1 = state_dim[0] // 4 * state_dim[1] // 4 * num_feature_maps_c2 #For a 2-Pooling layers of 2x2
		#~ 
		#~ W_conv1 = tf.Variable(tf.truncated_normal([5, 5, state_dim[2], num_feature_maps_c1], stddev=0.1))
		#~ b_conv1 =  tf.Variable(tf.constant(0.1, shape=[num_feature_maps_c1]))
		#~ W_conv2 = tf.Variable(tf.truncated_normal([5, 5, num_feature_maps_c1, num_feature_maps_c2], stddev=0.1))
		#~ b_conv2 = tf.Variable(tf.constant(0.1, shape=[num_feature_maps_c2]))
 		#~ W_fc1 = self.variable([input_dim_fc1, NUM_HIDDEN_UNITS_fc1], input_dim_fc1)
		#~ b_fc1 = self.variable([NUM_HIDDEN_UNITS_fc1], input_dim_fc1)
		#~ W_fc2 = tf.Variable(tf.random_uniform([NUM_HIDDEN_UNITS_fc1, action_dim], 3e-4, 3e-4))
		#~ b_fc2 = tf.Variable(tf.random_uniform([action_dim], 3e-4, 3e-4))
#~ 
		#~ h_conv1 = tf.nn.relu(tf.nn.conv2d(state_input_image, W_conv1, strides=[1, 1, 1, 1], padding='SAME') + b_conv1)
		#~ h_pool1 = tf.nn.max_pool(h_conv1, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding='SAME')
		#~ h_conv2 = tf.nn.relu(tf.nn.conv2d(h_pool1, W_conv2, strides=[1, 1, 1, 1], padding='SAME') + b_conv2)
		#~ h_pool2 = tf.nn.max_pool(h_conv2, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding='SAME')
		#~ h_pool2_flat = tf.reshape(h_pool2, [-1, input_dim_fc1])
		#~ h_fc1 = tf.nn.relu(tf.matmul(h_pool2_flat, W_fc1) + b_fc1)
		#~ action_output = tf.tanh(tf.matmul(h_fc1, W_fc2) + b_fc2)
        #~ return state_input,action_output,[W_conv1, b_conv1, W_conv2, b_conv2, W_fc1, b_fc1, W_fc2, b_fc2]

		state_input = tf.placeholder("float",[None,state_dim])
		is_training = tf.placeholder(tf.bool)

		W1 = self.variable([state_dim,self.layer1_size],state_dim)
		b1 = self.variable([self.layer1_size],state_dim)
		W2 = self.variable([self.layer1_size,self.layer2_size],self.layer1_size)
		b2 = self.variable([self.layer2_size],self.layer1_size)
		W3 = tf.Variable(tf.random_uniform([self.layer2_size,action_dim],-3e-3,3e-3))
		b3 = tf.Variable(tf.random_uniform([action_dim],-3e-3,3e-3))

		layer0_bn = self.batch_norm_layer(state_input,training_phase=is_training,scope_bn='batch_norm_0',activation=tf.identity)
		layer1 = tf.matmul(layer0_bn,W1) + b1
		layer1_bn = self.batch_norm_layer(layer1,training_phase=is_training,scope_bn='batch_norm_1',activation=tf.nn.relu)
		layer2 = tf.matmul(layer1_bn,W2) + b2
		layer2_bn = self.batch_norm_layer(layer2,training_phase=is_training,scope_bn='batch_norm_2',activation=tf.nn.relu)

		action_output = tf.tanh(tf.matmul(layer2_bn,W3) + b3)

		# Scaling the output
		action_limits = [None]*len(self.action_max)
		for i in range(len(self.action_max)):
			action_limits[i] = self.action_max[i]

		w = tf.constant(action_limits)

		action_output_limited = tf.multiply(action_output, w)

		return state_input,action_output_limited,[W1,b1,W2,b2,W3,b3],is_training

	def create_target_network(self,state_dim,action_dim,net):
		state_input = tf.placeholder("float",[None,state_dim])
		is_training = tf.placeholder(tf.bool)
		ema = tf.train.ExponentialMovingAverage(decay=1-self.tau)
		target_update = ema.apply(net)
		target_net = [ema.average(x) for x in net]

		layer0_bn = self.batch_norm_layer(state_input,training_phase=is_training,scope_bn='target_batch_norm_0',activation=tf.identity)

		layer1 = tf.matmul(layer0_bn,target_net[0]) + target_net[1]
		layer1_bn = self.batch_norm_layer(layer1,training_phase=is_training,scope_bn='target_batch_norm_1',activation=tf.nn.relu)
		layer2 = tf.matmul(layer1_bn,target_net[2]) + target_net[3]
		layer2_bn = self.batch_norm_layer(layer2,training_phase=is_training,scope_bn='target_batch_norm_2',activation=tf.nn.relu)

		action_output = tf.tanh(tf.matmul(layer2_bn,target_net[4]) + target_net[5])

		# Scaling the output
		action_limits = [None]*len(self.action_max)
		for i in range(len(self.action_max)):
			action_limits[i] = self.action_max[i]

		w = tf.constant(action_limits)

		action_output_limited = tf.multiply(action_output, w)

		return state_input,action_output_limited,target_update,is_training

	def update_target(self):
		self.sess.run(self.target_update)

	def train(self,q_gradient_batch,state_batch):
		self.sess.run(self.optimizer,feed_dict={
			self.q_gradient_input:q_gradient_batch,
			self.state_input:state_batch,
			self.is_training: True
			})

	def actions(self,state_batch):
		return self.sess.run(self.action_output,feed_dict={
			self.state_input:state_batch,
			self.is_training: True
			})

	def action(self,state):
		return self.sess.run(self.action_output,feed_dict={
			self.state_input:[state],
			self.is_training: False
			})[0]


	def target_actions(self,state_batch):
		return self.sess.run(self.target_action_output,feed_dict={
			self.target_state_input: state_batch,
			self.target_is_training: True
			})

	# f fan-in size
	def variable(self,shape,f):
		return tf.Variable(tf.random_uniform(shape,-1/math.sqrt(f),1/math.sqrt(f)))


	def batch_norm_layer(self,x,training_phase,scope_bn,activation=None):
		return tf.cond(training_phase, 
		lambda: tf.contrib.layers.batch_norm(x, activation_fn=activation, center=True, scale=True,
		updates_collections=None,is_training=True, reuse=None,scope=scope_bn,decay=0.9, epsilon=1e-5),
		lambda: tf.contrib.layers.batch_norm(x, activation_fn =activation, center=True, scale=True,
		updates_collections=None,is_training=False, reuse=True,scope=scope_bn,decay=0.9, epsilon=1e-5))

	def save_network(self,time_step, mydir):
		print 'save actor-network...',time_step
		self.saver.save(self.sess, mydir + '/' + 'actor-network' + str(time_step))


	def load_network(self):
		self.saver = tf.train.Saver(max_to_keep=15)
		checkpoint = tf.train.get_checkpoint_state("saved_actor_networks")
		if checkpoint and checkpoint.model_checkpoint_path:
			print "Restoring network from path: ", checkpoint.model_checkpoint_path
			self.saver.restore(self.sess, checkpoint.model_checkpoint_path)
			print "Successfully loaded:", checkpoint.model_checkpoint_path
		else:
			print "Could not find old network weights"





		
