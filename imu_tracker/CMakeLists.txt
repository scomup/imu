cmake_minimum_required(VERSION 2.8.12)  # Ships with Ubuntu 14.04 (Trusty)

project(imu_tracker)

set(CMAKE_BUILD_TYPE debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0")


find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs  message_filters laser_geometry  rosbag)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})

#message(FATAL_ERROR ${catkin_INCLUDE_DIRS})

file(GLOB_RECURSE ALL_SRCS "*.cc")
file(GLOB_RECURSE REMOVE_CMAKE "build/*")
list(REMOVE_ITEM ALL_SRCS ${REMOVE_CMAKE})

add_executable(${PROJECT_NAME}  ${ALL_SRCS})


target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Pangolin_LIBRARIES}
    pthread)
