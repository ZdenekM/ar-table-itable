cmake_minimum_required(VERSION 2.8.3)
project(art_user_tracking)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

set(CAFFE_URL "http://posefs1.perception.cs.cmu.edu/Users/ZheCao/pose_iter_440000.caffemodel")
set(CAFFE_DATA_PATH ${CMAKE_CURRENT_SOURCE_DIR}/data/pose_iter_440000.caffemodel)

if (NOT EXISTS "${CAFFE_DATA_PATH}")
  message(STATUS "Downloading caffemodel...")
  file(
    DOWNLOAD "${CAFFE_URL}" "${CAFFE_DATA_PATH}"
    SHOW_PROGRESS
  )
endif()

set(PROTO_URL "https://raw.githubusercontent.com/ZheC/Realtime_Multi-Person_Pose_Estimation/master/model/_trained_COCO/pose_deploy.prototxt")
set(PROTO_DATA_PATH ${CMAKE_CURRENT_SOURCE_DIR}/data/pose_deploy.prototxt)

if (NOT EXISTS "${PROTO_DATA_PATH}")
  message(STATUS "Downloading proto...")
  file(
    DOWNLOAD "${PROTO_URL}" "${PROTO_DATA_PATH}"
    SHOW_PROGRESS
  )
endif()

set(CONFIG_URL "https://raw.githubusercontent.com/ZheC/Realtime_Multi-Person_Pose_Estimation/master/testing/python/config")
set(CONFIG_DATA_PATH ${CMAKE_CURRENT_SOURCE_DIR}/data/config)

if (NOT EXISTS "${CONFIG_DATA_PATH}")
  message(STATUS "Downloading config...")
  file(
    DOWNLOAD "${CONFIG_URL}" "${CONFIG_DATA_PATH}"
    SHOW_PROGRESS
  )
endif()




catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES art_user_tracking
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
