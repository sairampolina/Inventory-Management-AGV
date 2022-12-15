cmake_minimum_required(VERSION 3.0.2)
project(im_agv)

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
#find_package(OpenCV COMPONENTS opencv_highgui opencv_features2d opencv_xfeatures2d opencv_calib3d)
#set(OpenCV_FOUND 1)
#if(NOT OPENCV_FOUND)
find_package(OpenCV REQUIRED COMPONENTS opencv_highgui opencv_features2d opencv_calib3d)
add_definitions(-DNO_CV_XFEATURES2D)
#endif()

find_package(Eigen3 REQUIRED)
catkin_package(
)


# build files 

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

add_library(tiago_lib src/my_tiago.cpp)
add_library(navigation_lib src/navigation.cpp)
add_library(perception_lib src/perception.cpp)
add_library(manipulation_lib src/manipulation.cpp)
add_library(object_lib src/object.cpp)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tiago_node src/my_tiago.cpp src/navigation.cpp 
          src/perception.cpp src/manipulation.cpp)

target_link_libraries(tiago_node 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)


add_executable(object_node src/object.cpp)

target_link_libraries(object_node
  ${catkin_LIBRARIES}
)

# Testing

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_im_agv.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(main_rostest
    test/main_test.test
    test/main.cpp
    
    test/test_perception.cpp
    test/test_navigation.cpp
#    test/detect_object_test.cpp
#    test/grasp_object_test.cpp
#    test/test_object.cpp
  )


  target_link_libraries(main_rostest
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}


    tiago_lib
    navigation_lib
    perception_lib
    detector_lib
    object_lib
    
    )
endif()

