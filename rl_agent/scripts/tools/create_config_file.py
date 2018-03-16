
from configobj import ConfigObj

config = ConfigObj()
config.filename = '../configs/config.ini'
config['DDPG'] = {}
config['DDPG']['Actor'] = {}
config['DDPG']['Actor']['LAYER1_SIZE'] = 300
config['DDPG']['Actor']['LAYER2_SIZE'] = 200
config['DDPG']['Actor']['LEARNING_RATE'] = 1e-4
config['DDPG']['Actor']['TAU'] = 0.001
config['DDPG']['Actor']['BATCH_SIZE'] = 64

config['DDPG']['Critic'] = {}
config['DDPG']['Critic']['LAYER1_SIZE'] = 300
config['DDPG']['Critic']['LAYER2_SIZE'] = 200
config['DDPG']['Critic']['LEARNING_RATE'] = 1e-3
config['DDPG']['Critic']['TAU'] = 0.001
config['DDPG']['Critic']['L2'] = 0.01
config.write()
