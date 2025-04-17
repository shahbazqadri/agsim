# Install script for directory: /home/wmf-admin/Desktop/inferencerobotcontrolpkg/src/motoman/motoman_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/install" TYPE PROGRAM FILES "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/build/motoman_msgs/catkin_generated/installspace/_setup_util.py")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/install" TYPE PROGRAM FILES "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/build/motoman_msgs/catkin_generated/installspace/env.sh")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/install/setup.bash;/home/wmf-admin/Desktop/inferencerobotcontrolpkg/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/install" TYPE FILE FILES
    "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/build/motoman_msgs/catkin_generated/installspace/setup.bash"
    "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/build/motoman_msgs/catkin_generated/installspace/local_setup.bash"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/install/setup.sh;/home/wmf-admin/Desktop/inferencerobotcontrolpkg/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/install" TYPE FILE FILES
    "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/build/motoman_msgs/catkin_generated/installspace/setup.sh"
    "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/build/motoman_msgs/catkin_generated/installspace/local_setup.sh"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/install/setup.zsh;/home/wmf-admin/Desktop/inferencerobotcontrolpkg/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/install" TYPE FILE FILES
    "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/build/motoman_msgs/catkin_generated/installspace/setup.zsh"
    "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/build/motoman_msgs/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/install" TYPE FILE FILES "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/build/motoman_msgs/catkin_generated/installspace/.rosinstall")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/motoman_msgs/msg" TYPE FILE FILES
    "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/src/motoman/motoman_msgs/msg/DynamicJointPoint.msg"
    "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/src/motoman/motoman_msgs/msg/DynamicJointsGroup.msg"
    "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/src/motoman/motoman_msgs/msg/DynamicJointState.msg"
    "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/src/motoman/motoman_msgs/msg/DynamicJointTrajectory.msg"
    "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/src/motoman/motoman_msgs/msg/DynamicJointTrajectoryFeedback.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/motoman_msgs/srv" TYPE FILE FILES
    "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/src/motoman/motoman_msgs/srv/CmdJointTrajectoryEx.srv"
    "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/src/motoman/motoman_msgs/srv/ReadSingleIO.srv"
    "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/src/motoman/motoman_msgs/srv/WriteSingleIO.srv"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/motoman_msgs/cmake" TYPE FILE FILES "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/build/motoman_msgs/catkin_generated/installspace/motoman_msgs-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/devel/.private/motoman_msgs/include/motoman_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/devel/.private/motoman_msgs/share/roseus/ros/motoman_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/devel/.private/motoman_msgs/share/common-lisp/ros/motoman_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/devel/.private/motoman_msgs/share/gennodejs/ros/motoman_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/devel/.private/motoman_msgs/lib/python2.7/dist-packages/motoman_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/devel/.private/motoman_msgs/lib/python2.7/dist-packages/motoman_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/build/motoman_msgs/catkin_generated/installspace/motoman_msgs.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/motoman_msgs/cmake" TYPE FILE FILES "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/build/motoman_msgs/catkin_generated/installspace/motoman_msgs-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/motoman_msgs/cmake" TYPE FILE FILES
    "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/build/motoman_msgs/catkin_generated/installspace/motoman_msgsConfig.cmake"
    "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/build/motoman_msgs/catkin_generated/installspace/motoman_msgsConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/motoman_msgs" TYPE FILE FILES "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/src/motoman/motoman_msgs/package.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/wmf-admin/Desktop/inferencerobotcontrolpkg/build/motoman_msgs/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/wmf-admin/Desktop/inferencerobotcontrolpkg/build/motoman_msgs/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
