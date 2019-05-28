# Copyright (C) 2017 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Gonzalez Jonas
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 3.5)
SET(KEYWORD objectTracking)
set(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
project(${KEYWORD})

FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(ICUBcontrib REQUIRED)


list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(YarpInstallationHelpers)
include(ICUBcontribHelpers)
include(ICUBcontribOptions)


icubcontrib_set_default_prefix()
set(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}" CACHE PATH "Application path prefix")

#search for includes
INCLUDE_DIRECTORIES(
        ${PROJECT_SOURCE_DIR}/include
        ${YARP_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        /usr/local/src/robot/yarp/src/libYARP_cv/include
)

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c include/iCub/kfebt/trackers/*.cpp include/iCub/kfebt/trackers/*/*.cpp include/iCub/kfebt/trackers/*/*/*.cpp)
FILE(GLOB folder_header include/iCub/*.h include/iCub/kfebt/*.h include/iCub/kfebt/trackers/*.h include/iCub/kfebt/trackers/*/*.h include/iCub/kfebt/trackers/*/*/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})


set(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}" CACHE PATH "Application path prefix")
# Set up our main executable
IF (folder_source)
    ADD_EXECUTABLE(${KEYWORD}
            ${folder_source}
            ${folder_header}
            )

    TARGET_LINK_LIBRARIES( ${KEYWORD}
            ${YARP_LIBRARIES}
            ${OpenCV_LIBS}
            )

    install(TARGETS  ${KEYWORD} DESTINATION bin)


ELSE (folder_source)
    MESSAGE( "No source code files found. Please add something")
ENDIF (folder_source)


icubcontrib_add_uninstall_target()

add_subdirectory(app)
