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

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if (CMAKE_COMPILER_IS_GNUXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rosidl_default_generators REQUIRED)

set(MSG_FILES
  "msg/ConfData.msg"
  "msg/ExtJoint.msg"
  "msg/JointTarget.msg"
  "msg/LoadData.msg"
  "msg/Orient.msg"
  "msg/Pos.msg"
  "msg/Pose.msg"
  "msg/RobJoint.msg"
  "msg/RobTarget.msg"
  "msg/ToolData.msg"
  "msg/WObjData.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
)

ament_export_dependencies(rosidl_default_generators)
ament_package()
