cmake_minimum_required(VERSION 3.0.2)
project(itri)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  sensor_msgs
  nav_msgs
  rospy
  tf
  image_transport
  visualization_msgs
  cv_bridge
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hw11
#  CATKIN_DEPENDS roscpp rospy std_msgs
DEPENDS EIGEN3 PCL
)

#inlude myself library
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(initial 
src/initial.cpp
src/process.cpp
src/parameter.cpp)
target_link_libraries(initial ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(try src/try.cpp)
target_link_libraries(try ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
