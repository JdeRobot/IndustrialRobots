cmake_minimum_required(VERSION 3.5)

########################################################################################################################
# Metadata
########################################################################################################################
# Read the package manifest.
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/package.xml" package_xml_str)

# Extract project name.
if(NOT package_xml_str MATCHES "<name>([A-Za-z0-9_]+)</name>")
  message(FATAL_ERROR "Could not parse project name from package manifest (aborting)")
else()
  set(extracted_name ${CMAKE_MATCH_1})
endif()

# Extract project version.
if(NOT package_xml_str MATCHES "<version>([0-9]+.[0-9]+.[0-9]+)</version>")
  message(FATAL_ERROR "Could not parse project version from package manifest (aborting)")
else()
  set(extracted_version ${CMAKE_MATCH_1})
endif()

########################################################################################################################
# CMake project
########################################################################################################################
project(${extracted_name} VERSION ${extracted_version})

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(MSG_FILES
  "msg/MechanicalUnitState.msg"
  "msg/RAPIDSymbolPath.msg"
  "msg/RAPIDTaskState.msg"
  "msg/ServiceResponses.msg"
  "msg/SystemState.msg"
)

set(SRV_FILES
  "srv/GetFileContents.srv"
  "srv/GetIOSignal.srv"
  "srv/GetRAPIDBool.srv"
  "srv/GetRAPIDDnum.srv"
  "srv/GetRAPIDNum.srv"
  "srv/GetRAPIDString.srv"
  "srv/GetRAPIDSymbol.srv"
  "srv/GetRobotControllerDescription.srv"
  "srv/GetSpeedRatio.srv"
  "srv/SetFileContents.srv"
  "srv/SetIOSignal.srv"
  "srv/SetRAPIDBool.srv"
  "srv/SetRAPIDDnum.srv"
  "srv/SetRAPIDNum.srv"
  "srv/SetRAPIDString.srv"
  "srv/SetRAPIDSymbol.srv"
  "srv/SetSpeedRatio.srv"
  "srv/TriggerWithResultCode.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  DEPENDENCIES std_msgs
  ADD_LINTER_TESTS
)
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
