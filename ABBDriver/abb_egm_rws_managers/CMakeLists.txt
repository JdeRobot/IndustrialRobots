cmake_minimum_required(VERSION 3.9.1)

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
project(${extracted_name} VERSION ${extracted_version} LANGUAGES CXX)

#===========================================================
# Dependencies
#===========================================================
#-----------------------------
# RWS and EGM (interfaces to
# ABB robot controllers)
#-----------------------------
find_package(abb_libegm REQUIRED)
find_package(abb_librws REQUIRED)

#-----------------------------
# Threads (determines the
# system's thread library)
#-----------------------------
find_package(Threads REQUIRED)

#-----------------------------
# Google protocol buffers
#-----------------------------
# Temporary workaround for https://github.com/ms-iot/ROSOnWindows/issues/218.
if(WIN32)
  foreach(prefix IN ITEMS $ENV{CMAKE_PREFIX_PATH})
    if(${prefix} STREQUAL "C:/opt/rosdeps/x64")
      list(APPEND CMAKE_PROGRAM_PATH "C:/opt/rosdeps/x64/tools/protobuf")
    endif()
  endforeach()
endif()

find_package(Protobuf REQUIRED)

# Workaround Protobuf exporting 'lpthread' as a library, the
# Threads package determines the system's thread library.
list(REMOVE_ITEM PROTOBUF_LIBRARIES -lpthread)

# Verify that the Protobuf compiler 'protoc' is present, since
# the above find_package() doesn't appear to check that.
if(NOT PROTOBUF_PROTOC_EXECUTABLE)
  message(FATAL_ERROR "Cannot find required 'protoc', cannot process Protobuf files without it (aborting)")
endif()

set(proto_files proto/robot_controller_description.proto)

if(NOT QUIET)
  message(STATUS "Generating protobuf C++ for: ${proto_files}")
endif()

# Generate C++ files for Protobuf message definitions, C++ headers and
# sources are written to the CMAKE_CURRENT_BINARY_DIR location.
if(MSVC)
  # Add export macro when using Microsoft Visual C++ compiler.
  protobuf_generate_cpp(proto_sources proto_headers EXPORT_MACRO ABB_EGM_RWS_MANAGERS_EXPORT ${proto_files})
else()
  protobuf_generate_cpp(proto_sources proto_headers ${proto_files})
endif()

#===========================================================
# Settings
#===========================================================
if(NOT DEFINED BUILD_SHARED_LIBS)
  set(BUILD_SHARED_LIBS ON)
endif()

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message(STATUS "${PROJECT_NAME}: Defaulting build type to RelWithDebInfo")
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

#===========================================================
# Build targets
#===========================================================
include(GenerateExportHeader)

#-----------------------------
# Library target
#-----------------------------
set(
  src_files
    src/egm_manager.cpp
    src/rws_manager.cpp
    src/system_data_parser.cpp
    src/utilities.cpp
    "${proto_sources}"
)

add_library(${PROJECT_NAME} ${src_files})
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

# Generate a header with export macros, which is written
# to the CMAKE_CURRENT_BINARY_DIR location.
generate_export_header(${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/include>"
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  abb_libegm::abb_libegm
  abb_librws::abb_librws
  protobuf::libprotobuf
  Threads::Threads
)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_14)

target_compile_definitions(${PROJECT_NAME} PRIVATE -D_USE_MATH_DEFINES)

if(NOT BUILD_SHARED_LIBS)
  target_compile_definitions(${PROJECT_NAME} PUBLIC "ABB_EGM_RWS_MANAGERS_STATIC_DEFINE")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
  target_compile_options(${PROJECT_NAME} PRIVATE /W4)
endif()

if(MSVC)
  # Force include the export header when using Microsoft Visual C++ compiler.
  target_compile_options(${PROJECT_NAME} PUBLIC "/FI${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_export.h")
endif()

#===========================================================
# CMake package configurations
#===========================================================
include(CMakePackageConfigHelpers)

# Create the ${PROJECT_NAME}Config.cmake file, which is used by
# other packages to find this package and its dependencies.
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Config.cmake.in"
  "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake"
  @ONLY
)

# Create the ${PROJECT_NAME}ConfigVersion.cmake.
write_basic_package_version_file(
  "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}ConfigVersion.cmake"
  COMPATIBILITY AnyNewerVersion
)

#===========================================================
# Install
#===========================================================
include(GNUInstallDirs)

install(
  DIRECTORY include/
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)

install(
  FILES
    "${proto_files}"
    "${proto_headers}"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_export.h"
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}"
)

install(
  FILES
    "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake"
    "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}ConfigVersion.cmake"
    "${CMAKE_CURRENT_SOURCE_DIR}/package.xml"
  DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}"
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT EXPORT_${PROJECT_NAME}
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
)

install(
  EXPORT EXPORT_${PROJECT_NAME}
  DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}"
  NAMESPACE ${PROJECT_NAME}::
  FILE ${PROJECT_NAME}Targets.cmake
)

#===========================================================
# Export targets
#===========================================================
export(
  EXPORT EXPORT_${PROJECT_NAME}
  NAMESPACE ${PROJECT_NAME}::
  FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake"
)
