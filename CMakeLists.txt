cmake_minimum_required(VERSION 3.0.2)
project(auto_husky)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  laser_geometry
  message_filters
  message_generation
  rosbag
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

find_package(PCL 1.7 REQUIRED COMPONENTS common io visualization filters segmentation sample_consensus)

add_message_files(
  FILES
  avoidance_input.msg
)
generate_messages(DEPENDENCIES std_msgs)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES auto_husky
#  CATKIN_DEPENDS cv_bridge image_transport laser_geometry message_filters message_generation rosbag roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories( ${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(particle_tracking src/particle_tracking.cpp)
target_link_libraries(particle_tracking ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_executable(data_play src/data_play.cpp)
target_link_libraries(data_play ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})
