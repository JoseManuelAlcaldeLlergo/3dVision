CMAKE_MINIMUM_REQUIRED(VERSION 3.1)
PROJECT(img_equalization)
ENABLE_LANGUAGE(CXX)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS_DEBUG "-ggdb3 -O0 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-g -O3 -Wall")

FIND_PACKAGE(OpenCV 3.4	REQUIRED )
LINK_LIBRARIES(${OpenCV_LIBS})
include_directories ("${OpenCV_INCLUDE_DIRS}")

add_executable(img_equalization img_equalization.cpp common_code.cpp
    common_code.hpp)

add_executable(test_common_code test_common_code_obf.cpp common_code.cpp
    common_code.hpp)

