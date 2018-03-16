cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME rl_boost_python)
project(${PROJECT_NAME})

add_definitions(-std=c++11)


# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries


set(RL_BOOST_PYTHON_SOURCE_DIR src/sources)
set(RL_BOOST_PYTHON_INCLUDE_DIR src/include)
set(RL_BOOST_PYTHON_TEST_DIR src/test)

FILE(GLOB_RECURSE RL_BOOST_PYTHON_HEADER_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/include/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/include/*.hpp)

FILE(GLOB_RECURSE RL_BOOST_PYTHON_SOURCE_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/sources/*.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/sources/*.cpp)

FILE(GLOB_RECURSE RL_BOOST_PYTHON_TEST_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/test/*.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/test/*.cpp)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
message(STATUS "CMAKE_MODULE_PATH is: " ${CMAKE_MODULE_PATH})

#find_package(PythonLibs 2.7 REQUIRED)
#LINK_DIRECTORIES(/usr/local/lib)
#OPTION(BUILD_SHARED_LIBS "turn OFF for .a libs" ON)

#find_package(OpenCV REQUIRED)
find_package(Boost 1.54 REQUIRED)
IF(Boost_FOUND)
  INCLUDE_DIRECTORIES("${Boost_INCLUDE_DIRS}" "/usr/include/python2.7")
  SET(Boost_USE_STATIC_LIBS OFF)
  SET(Boost_USE_MULTITHREADED ON)
  SET(Boost_USE_STATIC_RUNTIME OFF)
  FIND_PACKAGE(Boost 1.54 COMPONENTS python)


include_directories(${RL_BOOST_PYTHON_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIR})


add_library(rl_shm_python SHARED ${RL_BOOST_PYTHON_SOURCE_DIR}/rl_shm_python.cpp)
target_link_libraries(rl_shm_python ${Boost_LIBRARIES} rt)


ENDIF()

