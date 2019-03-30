cmake_minimum_required(VERSION 3.1)
project(pupil_comms)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -O3)

## note: This depends on the location of the current developer OpenCV installation folder
set(Eigen3_INCLUDE_DIRS /usr/include/eigen3)
find_package(OpenCV 3.2 REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(PC_ZeroMQ QUIET zmq)
find_path(ZeroMQ_INCLUDE_DIRS
    NAMES zmq.hpp
    PATHS ${PC_ZeroMQ_INCLUDE_DIRS}
)
find_library(ZeroMQ_LIBS
    NAMES zmq
    PATHS ${PC_ZeroMQ_LIBRARY_DIRS}
)

pkg_check_modules(PC_msgpack QUIET msgpack)
find_path(msgpack_INCLUDE_DIRS
    NAMES msgpack.h
    PATHS ${PC_ZeroMQ_INCLUDE_DIRS}
)
find_library(msgpack_LIBS
    NAMES msgpack
    PATHS ${PC_ZeroMQ_LIBRARY_DIRS}
)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    sensor_msgs
    gazebo_msgs
    geometry_msgs
    pupil_msgs
    ros_utils
    cv_bridge
    image_transport
    roscpp
    roslib
    rostopic
    gazebo_ros
    message_filters
    eigen_conversions
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
        std_msgs
        sensor_msgs
        message_filters
        gazebo_msgs
        geometry_msgs
        pupil_msgs
        ros_utils
        cv_bridge
        image_transport
        roscpp
        roslib
        rostopic
        gazebo_ros
        eigen_conversions
)

###########
## Build ##
###########


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${msgpack_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${ZeroMQ_INCLUDE_DIRS}
)

####============================================================================================####
####                           PUPIL_UTILS LIBRARY                                              ####
####============================================================================================####

set(LIBRARY_NAME "${PROJECT_NAME}")
set(CURRENT_NODE_DIR "src/nodelets")
file(GLOB CURRENT_NODE_SRC_FILES  "${CURRENT_NODE_DIR}/*.cc" )
add_library(${LIBRARY_NAME} ${CURRENT_NODE_SRC_FILES})
target_link_libraries( ${LIBRARY_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    realsense
    ${ZeroMQ_LIBS}
)
