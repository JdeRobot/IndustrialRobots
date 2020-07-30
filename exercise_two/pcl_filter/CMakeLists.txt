cmake_minimum_required(VERSION 3.0.2)
project(pcl_filter)

find_package(catkin REQUIRED COMPONENTS
  #pcl
  pcl_ros
  roscpp
  sensor_msgs
)

#set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl/")
find_package(PCL 1.7 REQUIRED COMPONENTS)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcl_filter
#  CATKIN_DEPENDS pcl pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(example src/example.cpp)
target_link_libraries(example ${catkin_LIBRARIES})
