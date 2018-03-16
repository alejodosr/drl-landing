#!/usr/bin/env python

from agent_ros_module_class import *
import rospy

def agent_ros_module_server():
    rospy.init_node('agent_ros_module')
    rate = rospy.Rate(20) # 20hz
    agent = AgentRosModule()
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
	agent_ros_module_server()


