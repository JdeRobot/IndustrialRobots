cmake_minimum_required(VERSION 3.0.2)
project(pcl_filter)

find_package(catkin REQUIRED COMPONENTS
  #pcl
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  message_generation
  pcl_filter_msgs
)

find_package(PCL 1.7 REQUIRED COMPONENTS)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#add_service_files(
#  DIRECTORY srv
#  FILES
#  color_filter.srv
#  shape_filter.srv
#)

#add_message_files(
#  FILES
#  color_filter.msg
#  shape_filter.msg
#)

#generate_messages(
#  DEPENDENCIES
#)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcl_filter
#  CATKIN_DEPENDS pcl pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
CATKIN_DEPENDS message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(example src/example.cpp)
target_link_libraries(example ${catkin_LIBRARIES})

add_executable(pcl_filter_server src/pcl_filter_server.cpp)
target_link_libraries(pcl_filter_server ${catkin_LIBRARIES})
