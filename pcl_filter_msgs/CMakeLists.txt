cmake_minimum_required(VERSION 3.8)
project(pcl_filter_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ColorFilter.msg"
  "msg/ShapeFilter.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

