CMAKE_MINIMUM_REQUIRED(VERSION 3.1)
PROJECT(sls)

# Set a default build type if none was specified
set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
      STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release")
endif()


ENABLE_LANGUAGE(CXX)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS_DEBUG "-ggdb3 -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-g0 -O3")

FIND_PACKAGE(OpenCV 3.4	REQUIRED )
LINK_LIBRARIES(${OpenCV_LIBS})
include_directories ("${OpenCV_INCLUDE_DIRS}")

set (LIB_SOURCES sls.hpp sls.cpp bc_scanning.hpp bc_scanning.cpp cparams.hpp cparams.cpp
    triangulation.hpp triangulation.cpp
    projector.hpp projector.cpp
    capturer.hpp capturer.cpp
    calibration.hpp calibration.cpp
    scanning_pattern_sequence.hpp scanning_pattern_sequence.cpp)

add_library(sls STATIC ${LIB_SOURCES})
add_executable(decode_bc_scanning decode_bc_scanning.cpp )
target_link_libraries(decode_bc_scanning sls)
add_executable(scan_pattern scan_pattern.cpp )
target_link_libraries(scan_pattern sls)
add_executable(mk_bc_scan mk_bc_scan.cpp )
target_link_libraries(mk_bc_scan sls)
add_executable(calibrate calibrate.cpp )
target_link_libraries(calibrate sls)

