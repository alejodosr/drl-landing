import tensorflow as tf
from tensorflow.contrib.layers.python.layers import batch_norm as batch_norm
import numpy as np
import math
import random

# Hyper Parameters
LAYER1_SIZE = 400
LAYER2_SIZE = 300

class TrainedActorNetwork:
    def __init__(self, env_info_list, actor_networks_path):

        self.path_to_saved_network = actor_networks_path
        self.sess = tf.InteractiveSession()

        self.state_dim = env_info_list[0]
        self.action_dim = env_info_list[4]
        self.action_min = env_info_list[5]
        self.action_max = env_info_list[6]

        # create actor network
        self.state_input, self.action_output, self.net, self.is_training = self.create_network(self.state_dim, self.action_dim)

        # self.sess.run(tf.initialize_all_variables())
        self.sess.run(tf.global_variables_initializer())

        # Load network
        self.load_network()

    def create_network(self, state_dim,action_dim):
        layer1_size = LAYER1_SIZE
        layer2_size = LAYER2_SIZE

        state_input = tf.placeholder("float", [None, state_dim])
        is_training = tf.placeholder(tf.bool)

        W1 = self.variable([state_dim, layer1_size], state_dim)
        b1 = self.variable([layer1_size], state_dim)
        W2 = self.variable([layer1_size, layer2_size], layer1_size)
        b2 = self.variable([layer2_size], layer1_size)
        W3 = tf.Variable(tf.random_uniform([layer2_size, action_dim], -3e-3, 3e-3))
        b3 = tf.Variable(tf.random_uniform([action_dim], -3e-3, 3e-3))

        layer0_bn = self.batch_norm_layer(state_input, training_phase=is_training, scope_bn='batch_norm_0',
                                          activation=tf.identity)
        layer1 = tf.matmul(layer0_bn, W1) + b1
        layer1_bn = self.batch_norm_layer(layer1, training_phase=is_training, scope_bn='batch_norm_1',
                                          activation=tf.nn.relu)
        layer2 = tf.matmul(layer1_bn, W2) + b2
        layer2_bn = self.batch_norm_layer(layer2, training_phase=is_training, scope_bn='batch_norm_2',
                                          activation=tf.nn.relu)

        action_output = tf.tanh(tf.matmul(layer2_bn, W3) + b3)

        # Scaling the output
        action_limits = [None] * len(self.action_max)
        for i in range(len(self.action_max)):
            action_limits[i] = self.action_max[i]

        w = tf.constant(action_limits)

        action_output_limited = tf.multiply(action_output, w)

        return state_input, action_output_limited, [W1, b1, W2, b2, W3, b3], is_training

    def action(self, state):
        return self.sess.run(self.action_output, feed_dict={
            self.state_input: [state],
            self.is_training: False
        })[0]

    # f fan-in size
    def variable(self, shape, f):
        return tf.Variable(tf.random_uniform(shape, -1 / math.sqrt(f), 1 / math.sqrt(f)))

    def batch_norm_layer(self, x, training_phase, scope_bn, activation=None):
        return tf.cond(training_phase,
                       lambda: tf.contrib.layers.batch_norm(x, activation_fn=activation, center=True, scale=True,
                                                            updates_collections=None, is_training=True, reuse=None,
                                                            scope=scope_bn, decay=0.9, epsilon=1e-5),
                       lambda: tf.contrib.layers.batch_norm(x, activation_fn=activation, center=True, scale=True,
                                                            updates_collections=None, is_training=False, reuse=True,
                                                            scope=scope_bn, decay=0.9, epsilon=1e-5))

    def load_network(self):
        self.saver = tf.train.Saver()
        checkpoint = tf.train.get_checkpoint_state(self.path_to_saved_network)
        if checkpoint and checkpoint.model_checkpoint_path:
            print "Restoring network from path: ", checkpoint.model_checkpoint_path
            self.saver.restore(self.sess, checkpoint.model_checkpoint_path)
            print "Successfully loaded:", checkpoint.model_checkpoint_path
        else:
            print "Could not find old network weights"
            
            
            
