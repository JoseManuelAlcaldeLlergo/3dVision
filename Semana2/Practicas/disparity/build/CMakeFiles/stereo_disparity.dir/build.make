# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pepe/UCO/V3D/Semana2/Practicas/disparity

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pepe/UCO/V3D/Semana2/Practicas/disparity/build

# Include any dependencies generated for this target.
include CMakeFiles/stereo_disparity.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/stereo_disparity.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/stereo_disparity.dir/flags.make

CMakeFiles/stereo_disparity.dir/stereo_disparity.cpp.o: CMakeFiles/stereo_disparity.dir/flags.make
CMakeFiles/stereo_disparity.dir/stereo_disparity.cpp.o: ../stereo_disparity.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pepe/UCO/V3D/Semana2/Practicas/disparity/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/stereo_disparity.dir/stereo_disparity.cpp.o"
	/bin/x86_64-linux-gnu-g++-9  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stereo_disparity.dir/stereo_disparity.cpp.o -c /home/pepe/UCO/V3D/Semana2/Practicas/disparity/stereo_disparity.cpp

CMakeFiles/stereo_disparity.dir/stereo_disparity.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stereo_disparity.dir/stereo_disparity.cpp.i"
	/bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pepe/UCO/V3D/Semana2/Practicas/disparity/stereo_disparity.cpp > CMakeFiles/stereo_disparity.dir/stereo_disparity.cpp.i

CMakeFiles/stereo_disparity.dir/stereo_disparity.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stereo_disparity.dir/stereo_disparity.cpp.s"
	/bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pepe/UCO/V3D/Semana2/Practicas/disparity/stereo_disparity.cpp -o CMakeFiles/stereo_disparity.dir/stereo_disparity.cpp.s

CMakeFiles/stereo_disparity.dir/common_code.cpp.o: CMakeFiles/stereo_disparity.dir/flags.make
CMakeFiles/stereo_disparity.dir/common_code.cpp.o: ../common_code.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pepe/UCO/V3D/Semana2/Practicas/disparity/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/stereo_disparity.dir/common_code.cpp.o"
	/bin/x86_64-linux-gnu-g++-9  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stereo_disparity.dir/common_code.cpp.o -c /home/pepe/UCO/V3D/Semana2/Practicas/disparity/common_code.cpp

CMakeFiles/stereo_disparity.dir/common_code.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stereo_disparity.dir/common_code.cpp.i"
	/bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pepe/UCO/V3D/Semana2/Practicas/disparity/common_code.cpp > CMakeFiles/stereo_disparity.dir/common_code.cpp.i

CMakeFiles/stereo_disparity.dir/common_code.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stereo_disparity.dir/common_code.cpp.s"
	/bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pepe/UCO/V3D/Semana2/Practicas/disparity/common_code.cpp -o CMakeFiles/stereo_disparity.dir/common_code.cpp.s

# Object files for target stereo_disparity
stereo_disparity_OBJECTS = \
"CMakeFiles/stereo_disparity.dir/stereo_disparity.cpp.o" \
"CMakeFiles/stereo_disparity.dir/common_code.cpp.o"

# External object files for target stereo_disparity
stereo_disparity_EXTERNAL_OBJECTS =

stereo_disparity: CMakeFiles/stereo_disparity.dir/stereo_disparity.cpp.o
stereo_disparity: CMakeFiles/stereo_disparity.dir/common_code.cpp.o
stereo_disparity: CMakeFiles/stereo_disparity.dir/build.make
stereo_disparity: /usr/local/lib/libopencv_dnn.so.3.4.16
stereo_disparity: /usr/local/lib/libopencv_highgui.so.3.4.16
stereo_disparity: /usr/local/lib/libopencv_ml.so.3.4.16
stereo_disparity: /usr/local/lib/libopencv_objdetect.so.3.4.16
stereo_disparity: /usr/local/lib/libopencv_shape.so.3.4.16
stereo_disparity: /usr/local/lib/libopencv_stitching.so.3.4.16
stereo_disparity: /usr/local/lib/libopencv_superres.so.3.4.16
stereo_disparity: /usr/local/lib/libopencv_videostab.so.3.4.16
stereo_disparity: /usr/local/lib/libopencv_calib3d.so.3.4.16
stereo_disparity: /usr/local/lib/libopencv_features2d.so.3.4.16
stereo_disparity: /usr/local/lib/libopencv_flann.so.3.4.16
stereo_disparity: /usr/local/lib/libopencv_photo.so.3.4.16
stereo_disparity: /usr/local/lib/libopencv_video.so.3.4.16
stereo_disparity: /usr/local/lib/libopencv_videoio.so.3.4.16
stereo_disparity: /usr/local/lib/libopencv_imgcodecs.so.3.4.16
stereo_disparity: /usr/local/lib/libopencv_imgproc.so.3.4.16
stereo_disparity: /usr/local/lib/libopencv_core.so.3.4.16
stereo_disparity: CMakeFiles/stereo_disparity.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pepe/UCO/V3D/Semana2/Practicas/disparity/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable stereo_disparity"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/stereo_disparity.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/stereo_disparity.dir/build: stereo_disparity

.PHONY : CMakeFiles/stereo_disparity.dir/build

CMakeFiles/stereo_disparity.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/stereo_disparity.dir/cmake_clean.cmake
.PHONY : CMakeFiles/stereo_disparity.dir/clean

CMakeFiles/stereo_disparity.dir/depend:
	cd /home/pepe/UCO/V3D/Semana2/Practicas/disparity/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pepe/UCO/V3D/Semana2/Practicas/disparity /home/pepe/UCO/V3D/Semana2/Practicas/disparity /home/pepe/UCO/V3D/Semana2/Practicas/disparity/build /home/pepe/UCO/V3D/Semana2/Practicas/disparity/build /home/pepe/UCO/V3D/Semana2/Practicas/disparity/build/CMakeFiles/stereo_disparity.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/stereo_disparity.dir/depend

