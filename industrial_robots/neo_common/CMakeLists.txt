cmake_minimum_required(VERSION 2.8.3)
project(neo_common)

set(CMAKE_CXX_STANDARD 11)

## By default build with code optimization and no debug info
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release"
  	  CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif(NOT CMAKE_BUILD_TYPE)

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
        )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS include
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
