cmake_minimum_required(VERSION 2.8.3)
project(neo_srvs)

## By default build with code optimization and no debug info
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release"
  	  CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif(NOT CMAKE_BUILD_TYPE)

find_package(catkin REQUIRED COMPONENTS message_generation)

add_service_files(FILES
  RelayBoardSetRelay.srv
  IOBoardSetDigOut.srv
  RelayBoardSetLCDMsg.srv
  LockPlatform.srv
  UnlockPlatform.srv
  ResetOmniWheels.srv
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)
