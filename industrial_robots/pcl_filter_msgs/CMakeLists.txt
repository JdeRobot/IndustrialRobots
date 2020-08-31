cmake_minimum_required(VERSION 3.0.2)
project(pcl_filter_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
  ColorFilter.msg
  ShapeFilter.msg
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcl_filter_msgs
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
CATKIN_DEPENDS message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
