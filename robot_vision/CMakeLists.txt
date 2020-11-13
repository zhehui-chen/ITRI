cmake_minimum_required(VERSION 2.8.3)
project(robot_vision)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_vision
#  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

add_executable(grayImage src/grayImage.cpp)
target_link_libraries(grayImage ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(init src/init.cpp src/parameter.cpp src/process.cpp)
target_link_libraries(init ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
