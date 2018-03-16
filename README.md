# A Deep Reinforcement Learning Technique for Autonomous Multirotor Landing on a Moving Platform

Parrot Bebop 2 autonomous landing on a moving platform. The behaviour has been completely learned in simulation without prior human knowledge and by means of deep reinforcement learning
techniques. Since the multirotor is controlled in attitude, no high level state estimation is required.

# Requirements
- Ubuntu 16.04 LTS
- ROS Kinetic
- OpenCV 3
- Tensorflow
- Boost (c++)
- Rospy
- CvBridge
- Matplotlib
- Numpy

# Installation and build

Create workspace and clone respository:

`mkdir ~/workspace/drl-landing/src`

`cd ~/workspace/drl-landing/src`

`git clone https://github.com/alejodosr/drl-landing`

Build ArUco library:

`cd ~/workspace/drl-landing/src/drl-landing/code-rl-environment-gazebo/rl_libs/aruco304`

`mkdir build`

`cd build`

`cmake ..`

`make - j4`

Build the workspace:

`cd ~/workspace/drl-landing`

`catkin_make`

(Download and install Parrot Bebop 2 Autonomy Driver under `~/workspace/drl-landing/src` - https://github.com/AutonomyLab/bebop_autonomy -)

# Usage

Coming soon..
