# Install script for directory: /home/alejandro/workspace/robot_reinforcement_learning/src/robot_reinforcement_learning/code-rl-environment-gazebo/rl_libs/aruco304/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so.3.0.4"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so.3.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE FILES
    "/home/alejandro/workspace/robot_reinforcement_learning/src/robot_reinforcement_learning/code-rl-environment-gazebo/rl_libs/aruco304/build/src/libaruco.so.3.0.4"
    "/home/alejandro/workspace/robot_reinforcement_learning/src/robot_reinforcement_learning/code-rl-environment-gazebo/rl_libs/aruco304/build/src/libaruco.so.3.0"
    "/home/alejandro/workspace/robot_reinforcement_learning/src/robot_reinforcement_learning/code-rl-environment-gazebo/rl_libs/aruco304/build/src/libaruco.so"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so.3.0.4"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so.3.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "/usr/local/cuda-8.0/lib64:/home/alejandro/sources/opencv-3.2.0/build/lib:"
           NEW_RPATH "")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/aruco" TYPE FILE FILES
    "/home/alejandro/workspace/robot_reinforcement_learning/src/robot_reinforcement_learning/code-rl-environment-gazebo/rl_libs/aruco304/src/aruco_export.h"
    "/home/alejandro/workspace/robot_reinforcement_learning/src/robot_reinforcement_learning/code-rl-environment-gazebo/rl_libs/aruco304/src/cameraparameters.h"
    "/home/alejandro/workspace/robot_reinforcement_learning/src/robot_reinforcement_learning/code-rl-environment-gazebo/rl_libs/aruco304/src/cvdrawingutils.h"
    "/home/alejandro/workspace/robot_reinforcement_learning/src/robot_reinforcement_learning/code-rl-environment-gazebo/rl_libs/aruco304/src/dictionary.h"
    "/home/alejandro/workspace/robot_reinforcement_learning/src/robot_reinforcement_learning/code-rl-environment-gazebo/rl_libs/aruco304/src/ippe.h"
    "/home/alejandro/workspace/robot_reinforcement_learning/src/robot_reinforcement_learning/code-rl-environment-gazebo/rl_libs/aruco304/src/marker.h"
    "/home/alejandro/workspace/robot_reinforcement_learning/src/robot_reinforcement_learning/code-rl-environment-gazebo/rl_libs/aruco304/src/markerdetector.h"
    "/home/alejandro/workspace/robot_reinforcement_learning/src/robot_reinforcement_learning/code-rl-environment-gazebo/rl_libs/aruco304/src/markerlabeler.h"
    "/home/alejandro/workspace/robot_reinforcement_learning/src/robot_reinforcement_learning/code-rl-environment-gazebo/rl_libs/aruco304/src/markermap.h"
    "/home/alejandro/workspace/robot_reinforcement_learning/src/robot_reinforcement_learning/code-rl-environment-gazebo/rl_libs/aruco304/src/posetracker.h"
    "/home/alejandro/workspace/robot_reinforcement_learning/src/robot_reinforcement_learning/code-rl-environment-gazebo/rl_libs/aruco304/src/markerlabelers/dictionary_based.h"
    "/home/alejandro/workspace/robot_reinforcement_learning/src/robot_reinforcement_learning/code-rl-environment-gazebo/rl_libs/aruco304/src/timers.h"
    "/home/alejandro/workspace/robot_reinforcement_learning/src/robot_reinforcement_learning/code-rl-environment-gazebo/rl_libs/aruco304/src/debug.h"
    "/home/alejandro/workspace/robot_reinforcement_learning/src/robot_reinforcement_learning/code-rl-environment-gazebo/rl_libs/aruco304/src/aruco.h"
    "/home/alejandro/workspace/robot_reinforcement_learning/src/robot_reinforcement_learning/code-rl-environment-gazebo/rl_libs/aruco304/src/markerlabelers/svmmarkers.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/aruco/cmake/arucoConfig.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/aruco/cmake/arucoConfig.cmake"
         "/home/alejandro/workspace/robot_reinforcement_learning/src/robot_reinforcement_learning/code-rl-environment-gazebo/rl_libs/aruco304/build/src/CMakeFiles/Export/share/aruco/cmake/arucoConfig.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/aruco/cmake/arucoConfig-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/aruco/cmake/arucoConfig.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aruco/cmake" TYPE FILE FILES "/home/alejandro/workspace/robot_reinforcement_learning/src/robot_reinforcement_learning/code-rl-environment-gazebo/rl_libs/aruco304/build/src/CMakeFiles/Export/share/aruco/cmake/arucoConfig.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aruco/cmake" TYPE FILE FILES "/home/alejandro/workspace/robot_reinforcement_learning/src/robot_reinforcement_learning/code-rl-environment-gazebo/rl_libs/aruco304/build/src/CMakeFiles/Export/share/aruco/cmake/arucoConfig-debug.cmake")
  endif()
endif()

