cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME rl_srvs)
project(${PROJECT_NAME})

add_definitions(-std=c++11)


# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries


set(RL_ENVIRONMENT_GAZEBO_ROS_SOURCE_DIR src/sources)
set(RL_ENVIRONMENT_GAZEBO_ROS_INCLUDE_DIR src/include)

FILE(GLOB_RECURSE RL_ENVIRONMENT_GAZEBO_ROS_HEADER_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/include/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/include/*.hpp)

FILE(GLOB_RECURSE RL_ENVIRONMENT_GAZEBO_ROS_SOURCE_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/sources/*.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/sources/*.cpp)


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
message(STATUS "CMAKE_MODULE_PATH is: " ${CMAKE_MODULE_PATH})


find_package(catkin REQUIRED
                COMPONENTS roscpp message_generation sensor_msgs)

add_service_files(
	FILES
	AgentSrv.srv
	EnvDimensionalitySrv.srv
	ResetEnvSrv.srv
	RecordExperimentSrv.srv
)

generate_messages(
	DEPENDENCIES
	roscpp
	sensor_msgs
)


catkin_package(
        CATKIN_DEPENDS roscpp message_runtime sensor_msgs
  )

include_directories(${RL_ENVIRONMENT_GAZEBO_ROS_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})

