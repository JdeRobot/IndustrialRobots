cmake_minimum_required(VERSION 3.8)
project(pcl_filter)

# Set CMake policy to suppress PCL_ROOT warning
cmake_policy(SET CMP0074 NEW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation features sample_consensus search)
find_package(std_srvs REQUIRED)

# Generate service interfaces (only if service files exist)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/srv/ColorFilter.srv" AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/srv/ShapeFilter.srv")
  rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/ColorFilter.srv"
    "srv/ShapeFilter.srv"
    DEPENDENCIES geometry_msgs std_msgs
  )
  set(HAS_SERVICES TRUE)
else()
  set(HAS_SERVICES FALSE)
  message(WARNING "Service files not found. Building without custom services.")
endif()

# Include directories
include_directories(${PCL_INCLUDE_DIRS})

# Add PCL definitions
add_definitions(${PCL_DEFINITIONS})

# Create executable
add_executable(pcl_filter_server src/pcl_filter_server.cpp)

# Target dependencies
ament_target_dependencies(pcl_filter_server
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  pcl_conversions
  cv_bridge
  OpenCV
)

# Link PCL libraries
target_link_libraries(pcl_filter_server ${PCL_LIBRARIES})

# Ensure service generation is complete before building (only if services exist)
if(HAS_SERVICES)
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(pcl_filter_server "${cpp_typesupport_target}")
endif()

# Install targets
install(TARGETS
  pcl_filter_server
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and config
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
