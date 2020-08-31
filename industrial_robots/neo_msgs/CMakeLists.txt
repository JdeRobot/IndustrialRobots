cmake_minimum_required(VERSION 2.8.3)
project(neo_msgs)

## By default build with code optimization and no debug info
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release"
  	  CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif(NOT CMAKE_BUILD_TYPE)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

 ##Generate messages in the 'msg' folder
 add_message_files(
   FILES
    	EmergencyStopState.msg
	Keypad.msg
	LCDOutput.msg
	USBoard.msg
	RelayBoardV2.msg
	IOBoard.msg
 )


 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  CATKIN_DEPENDS message_runtime
)


