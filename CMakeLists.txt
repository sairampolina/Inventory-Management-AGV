cmake_minimum_required(VERSION 3.0.2)
project(im_agv)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  moveit_msgs
  image_transport
  cv_bridge
  control_msgs
  actionlib
)

#find_package(OpenCV REQUIRED)
find_package(OpenCV COMPONENTS opencv_highgui opencv_features2d opencv_xfeatures2d opencv_calib3d)
if(NOT OPENCV_FOUND)
	find_package(OpenCV REQUIRED COMPONENTS opencv_highgui opencv_features2d opencv_calib3d)
	add_definitions(-DNO_CV_XFEATURES2D)
endif()

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES im_agv
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

add_library(my_tiago src/my_tiago.cpp)
add_library(navigation src/navigation.cpp)
add_library(packagedetector src/perception.cpp)
add_library(manipulator src/manipulation.cpp)
add_library(object src/object.cpp)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tiago_node src/my_tiago.cpp)

target_link_libraries(tiago_node
  navigation
  packagedetector
  manipulator
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(object_node src/object.cpp)

target_link_libraries(object_node
  ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_im_agv.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
