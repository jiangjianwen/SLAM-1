# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

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
CMAKE_COMMAND = /home/zpw/software/clion/clion-2019.2.2/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/zpw/software/clion/clion-2019.2.2/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/zpw/github/SLAM/Slambook2/ch08/useLK

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zpw/github/SLAM/Slambook2/ch08/useLK/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/useLK.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/useLK.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/useLK.dir/flags.make

CMakeFiles/useLK.dir/main.cpp.o: CMakeFiles/useLK.dir/flags.make
CMakeFiles/useLK.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zpw/github/SLAM/Slambook2/ch08/useLK/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/useLK.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/useLK.dir/main.cpp.o -c /home/zpw/github/SLAM/Slambook2/ch08/useLK/main.cpp

CMakeFiles/useLK.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/useLK.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zpw/github/SLAM/Slambook2/ch08/useLK/main.cpp > CMakeFiles/useLK.dir/main.cpp.i

CMakeFiles/useLK.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/useLK.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zpw/github/SLAM/Slambook2/ch08/useLK/main.cpp -o CMakeFiles/useLK.dir/main.cpp.s

# Object files for target useLK
useLK_OBJECTS = \
"CMakeFiles/useLK.dir/main.cpp.o"

# External object files for target useLK
useLK_EXTERNAL_OBJECTS =

useLK: CMakeFiles/useLK.dir/main.cpp.o
useLK: CMakeFiles/useLK.dir/build.make
useLK: /usr/local/opencv4/lib/libopencv_dnn.so.4.0.0
useLK: /usr/local/opencv4/lib/libopencv_ml.so.4.0.0
useLK: /usr/local/opencv4/lib/libopencv_objdetect.so.4.0.0
useLK: /usr/local/opencv4/lib/libopencv_photo.so.4.0.0
useLK: /usr/local/opencv4/lib/libopencv_stitching.so.4.0.0
useLK: /usr/local/opencv4/lib/libopencv_video.so.4.0.0
useLK: /usr/local/opencv4/lib/libopencv_calib3d.so.4.0.0
useLK: /usr/local/opencv4/lib/libopencv_features2d.so.4.0.0
useLK: /usr/local/opencv4/lib/libopencv_flann.so.4.0.0
useLK: /usr/local/opencv4/lib/libopencv_highgui.so.4.0.0
useLK: /usr/local/opencv4/lib/libopencv_videoio.so.4.0.0
useLK: /usr/local/opencv4/lib/libopencv_imgcodecs.so.4.0.0
useLK: /usr/local/opencv4/lib/libopencv_imgproc.so.4.0.0
useLK: /usr/local/opencv4/lib/libopencv_core.so.4.0.0
useLK: CMakeFiles/useLK.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zpw/github/SLAM/Slambook2/ch08/useLK/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable useLK"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/useLK.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/useLK.dir/build: useLK

.PHONY : CMakeFiles/useLK.dir/build

CMakeFiles/useLK.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/useLK.dir/cmake_clean.cmake
.PHONY : CMakeFiles/useLK.dir/clean

CMakeFiles/useLK.dir/depend:
	cd /home/zpw/github/SLAM/Slambook2/ch08/useLK/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zpw/github/SLAM/Slambook2/ch08/useLK /home/zpw/github/SLAM/Slambook2/ch08/useLK /home/zpw/github/SLAM/Slambook2/ch08/useLK/cmake-build-debug /home/zpw/github/SLAM/Slambook2/ch08/useLK/cmake-build-debug /home/zpw/github/SLAM/Slambook2/ch08/useLK/cmake-build-debug/CMakeFiles/useLK.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/useLK.dir/depend

