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
CMAKE_SOURCE_DIR = /home/pepe/UCO/V3D/color_balance

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pepe/UCO/V3D/color_balance/build

# Include any dependencies generated for this target.
include CMakeFiles/test_common_code.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/test_common_code.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test_common_code.dir/flags.make

CMakeFiles/test_common_code.dir/test_common_code_obf.cpp.o: CMakeFiles/test_common_code.dir/flags.make
CMakeFiles/test_common_code.dir/test_common_code_obf.cpp.o: ../test_common_code_obf.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pepe/UCO/V3D/color_balance/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/test_common_code.dir/test_common_code_obf.cpp.o"
	/bin/x86_64-linux-gnu-g++-9  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_common_code.dir/test_common_code_obf.cpp.o -c /home/pepe/UCO/V3D/color_balance/test_common_code_obf.cpp

CMakeFiles/test_common_code.dir/test_common_code_obf.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_common_code.dir/test_common_code_obf.cpp.i"
	/bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pepe/UCO/V3D/color_balance/test_common_code_obf.cpp > CMakeFiles/test_common_code.dir/test_common_code_obf.cpp.i

CMakeFiles/test_common_code.dir/test_common_code_obf.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_common_code.dir/test_common_code_obf.cpp.s"
	/bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pepe/UCO/V3D/color_balance/test_common_code_obf.cpp -o CMakeFiles/test_common_code.dir/test_common_code_obf.cpp.s

CMakeFiles/test_common_code.dir/common_code.cpp.o: CMakeFiles/test_common_code.dir/flags.make
CMakeFiles/test_common_code.dir/common_code.cpp.o: ../common_code.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pepe/UCO/V3D/color_balance/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/test_common_code.dir/common_code.cpp.o"
	/bin/x86_64-linux-gnu-g++-9  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_common_code.dir/common_code.cpp.o -c /home/pepe/UCO/V3D/color_balance/common_code.cpp

CMakeFiles/test_common_code.dir/common_code.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_common_code.dir/common_code.cpp.i"
	/bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pepe/UCO/V3D/color_balance/common_code.cpp > CMakeFiles/test_common_code.dir/common_code.cpp.i

CMakeFiles/test_common_code.dir/common_code.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_common_code.dir/common_code.cpp.s"
	/bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pepe/UCO/V3D/color_balance/common_code.cpp -o CMakeFiles/test_common_code.dir/common_code.cpp.s

# Object files for target test_common_code
test_common_code_OBJECTS = \
"CMakeFiles/test_common_code.dir/test_common_code_obf.cpp.o" \
"CMakeFiles/test_common_code.dir/common_code.cpp.o"

# External object files for target test_common_code
test_common_code_EXTERNAL_OBJECTS =

test_common_code: CMakeFiles/test_common_code.dir/test_common_code_obf.cpp.o
test_common_code: CMakeFiles/test_common_code.dir/common_code.cpp.o
test_common_code: CMakeFiles/test_common_code.dir/build.make
test_common_code: /usr/local/lib/libopencv_dnn.so.3.4.16
test_common_code: /usr/local/lib/libopencv_highgui.so.3.4.16
test_common_code: /usr/local/lib/libopencv_ml.so.3.4.16
test_common_code: /usr/local/lib/libopencv_objdetect.so.3.4.16
test_common_code: /usr/local/lib/libopencv_shape.so.3.4.16
test_common_code: /usr/local/lib/libopencv_stitching.so.3.4.16
test_common_code: /usr/local/lib/libopencv_superres.so.3.4.16
test_common_code: /usr/local/lib/libopencv_videostab.so.3.4.16
test_common_code: /usr/local/lib/libopencv_calib3d.so.3.4.16
test_common_code: /usr/local/lib/libopencv_features2d.so.3.4.16
test_common_code: /usr/local/lib/libopencv_flann.so.3.4.16
test_common_code: /usr/local/lib/libopencv_photo.so.3.4.16
test_common_code: /usr/local/lib/libopencv_video.so.3.4.16
test_common_code: /usr/local/lib/libopencv_videoio.so.3.4.16
test_common_code: /usr/local/lib/libopencv_imgcodecs.so.3.4.16
test_common_code: /usr/local/lib/libopencv_imgproc.so.3.4.16
test_common_code: /usr/local/lib/libopencv_core.so.3.4.16
test_common_code: CMakeFiles/test_common_code.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pepe/UCO/V3D/color_balance/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable test_common_code"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_common_code.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test_common_code.dir/build: test_common_code

.PHONY : CMakeFiles/test_common_code.dir/build

CMakeFiles/test_common_code.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test_common_code.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test_common_code.dir/clean

CMakeFiles/test_common_code.dir/depend:
	cd /home/pepe/UCO/V3D/color_balance/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pepe/UCO/V3D/color_balance /home/pepe/UCO/V3D/color_balance /home/pepe/UCO/V3D/color_balance/build /home/pepe/UCO/V3D/color_balance/build /home/pepe/UCO/V3D/color_balance/build/CMakeFiles/test_common_code.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test_common_code.dir/depend

