cmake_minimum_required(VERSION 3.8)
project(ur5_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch urdf srdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()