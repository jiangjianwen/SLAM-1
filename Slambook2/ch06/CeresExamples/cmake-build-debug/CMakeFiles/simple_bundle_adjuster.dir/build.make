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
CMAKE_SOURCE_DIR = /home/zpw/github/SLAM/Slambook2/ch06/CeresExamples

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zpw/github/SLAM/Slambook2/ch06/CeresExamples/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/simple_bundle_adjuster.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/simple_bundle_adjuster.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/simple_bundle_adjuster.dir/flags.make

CMakeFiles/simple_bundle_adjuster.dir/simple_bundle_adjuster.o: CMakeFiles/simple_bundle_adjuster.dir/flags.make
CMakeFiles/simple_bundle_adjuster.dir/simple_bundle_adjuster.o: ../simple_bundle_adjuster.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zpw/github/SLAM/Slambook2/ch06/CeresExamples/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/simple_bundle_adjuster.dir/simple_bundle_adjuster.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/simple_bundle_adjuster.dir/simple_bundle_adjuster.o -c /home/zpw/github/SLAM/Slambook2/ch06/CeresExamples/simple_bundle_adjuster.cc

CMakeFiles/simple_bundle_adjuster.dir/simple_bundle_adjuster.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/simple_bundle_adjuster.dir/simple_bundle_adjuster.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zpw/github/SLAM/Slambook2/ch06/CeresExamples/simple_bundle_adjuster.cc > CMakeFiles/simple_bundle_adjuster.dir/simple_bundle_adjuster.i

CMakeFiles/simple_bundle_adjuster.dir/simple_bundle_adjuster.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/simple_bundle_adjuster.dir/simple_bundle_adjuster.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zpw/github/SLAM/Slambook2/ch06/CeresExamples/simple_bundle_adjuster.cc -o CMakeFiles/simple_bundle_adjuster.dir/simple_bundle_adjuster.s

# Object files for target simple_bundle_adjuster
simple_bundle_adjuster_OBJECTS = \
"CMakeFiles/simple_bundle_adjuster.dir/simple_bundle_adjuster.o"

# External object files for target simple_bundle_adjuster
simple_bundle_adjuster_EXTERNAL_OBJECTS =

simple_bundle_adjuster: CMakeFiles/simple_bundle_adjuster.dir/simple_bundle_adjuster.o
simple_bundle_adjuster: CMakeFiles/simple_bundle_adjuster.dir/build.make
simple_bundle_adjuster: CMakeFiles/simple_bundle_adjuster.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zpw/github/SLAM/Slambook2/ch06/CeresExamples/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable simple_bundle_adjuster"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/simple_bundle_adjuster.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/simple_bundle_adjuster.dir/build: simple_bundle_adjuster

.PHONY : CMakeFiles/simple_bundle_adjuster.dir/build

CMakeFiles/simple_bundle_adjuster.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/simple_bundle_adjuster.dir/cmake_clean.cmake
.PHONY : CMakeFiles/simple_bundle_adjuster.dir/clean

CMakeFiles/simple_bundle_adjuster.dir/depend:
	cd /home/zpw/github/SLAM/Slambook2/ch06/CeresExamples/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zpw/github/SLAM/Slambook2/ch06/CeresExamples /home/zpw/github/SLAM/Slambook2/ch06/CeresExamples /home/zpw/github/SLAM/Slambook2/ch06/CeresExamples/cmake-build-debug /home/zpw/github/SLAM/Slambook2/ch06/CeresExamples/cmake-build-debug /home/zpw/github/SLAM/Slambook2/ch06/CeresExamples/cmake-build-debug/CMakeFiles/simple_bundle_adjuster.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/simple_bundle_adjuster.dir/depend
