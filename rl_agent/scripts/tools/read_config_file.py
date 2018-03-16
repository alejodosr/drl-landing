
from configobj import ConfigObj
config = ConfigObj('../configs/config.ini')
Actor = config['DDPG']['Actor']
LAYER1_SIZE = Actor['LAYER1_SIZE']
LAYER2_SIZE = Actor['LAYER2_SIZE']
LEARNING_RATE = Actor['LEARNING_RATE']
TAU = Actor['TAU']
BATCH_SIZE = Actor['BATCH_SIZE']

print 'LAYER1_SIZE: ', LAYER1_SIZE
print 'LAYER2_SIZE: ', LAYER2_SIZE
print 'LEARNING_RATE: ', LEARNING_RATE
print 'TAU: ', TAU
print 'BATCH_SIZE: ', BATCH_SIZE
