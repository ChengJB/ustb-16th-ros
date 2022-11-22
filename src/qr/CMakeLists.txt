cmake_minimum_required(VERSION 3.0.2)
project(qr)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  qr_msgs
)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES qr
   CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs
   qr_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  #/usr/include/eigen3
)

add_executable(qr_node src/qr.cpp)

target_link_libraries(qr_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

# add_dependencies(qr_node ${catkin_EXPORTED_TARGETS} qr_msgs)
