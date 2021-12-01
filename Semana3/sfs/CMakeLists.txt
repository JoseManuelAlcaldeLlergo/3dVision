CMAKE_MINIMUM_REQUIRED(VERSION 3.1)
PROJECT(sfs)
ENABLE_LANGUAGE(CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS_DEBUG "-ggdb3 -O0 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-g -O3 -Wall")

set(WITH_OPENMP OFF CACHE BOOL "Use openmp for pararell processing.")
if (WITH_OPENMP)
  find_package(OpenMP REQUIRED)
  if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    add_definitions(-DUSE_OPENMP)
  endif(OPENMP_FOUND)
endif (WITH_OPENMP)

FIND_PACKAGE(OpenCV 3.4	REQUIRED )
find_package(PkgConfig REQUIRED)
if (${PKG_CONFIG_FOUND})
  pkg_check_modules (ZLIB REQUIRED zlib)
endif()


LINK_LIBRARIES(${OpenCV_LIBS} ${ZLIB_LIBRARIES})
include_directories (${OpenCV_INCLUDE_DIRS} ${ZLIB_INCLUDE_DIRS})
set (LIB_SOURCES code_todo.cpp sfs.hpp
    voxel.hpp voxel.cpp camera_parameters.hpp camera_parameters.cpp
    voxelset.hpp voxelset.cpp view.hpp view.cpp octree.hpp octree.cpp)

add_library(sfs STATIC ${LIB_SOURCES})
add_executable(mk_voxelset mk_voxelset.cpp)
target_link_libraries(mk_voxelset sfs)
add_executable(mk_octree mk_octree.cpp sfs)
target_link_libraries(mk_octree sfs)
add_executable(vs2wrl vs2wrl.cpp sfs)
target_link_libraries(vs2wrl sfs)
add_executable(oct2wrl oct2wrl.cpp sfs)
target_link_libraries(oct2wrl sfs)
