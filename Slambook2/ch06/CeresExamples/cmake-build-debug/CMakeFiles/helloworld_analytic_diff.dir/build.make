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
include CMakeFiles/helloworld_analytic_diff.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/helloworld_analytic_diff.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/helloworld_analytic_diff.dir/flags.make

CMakeFiles/helloworld_analytic_diff.dir/helloworld_analytic_diff.o: CMakeFiles/helloworld_analytic_diff.dir/flags.make
CMakeFiles/helloworld_analytic_diff.dir/helloworld_analytic_diff.o: ../helloworld_analytic_diff.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zpw/github/SLAM/Slambook2/ch06/CeresExamples/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/helloworld_analytic_diff.dir/helloworld_analytic_diff.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/helloworld_analytic_diff.dir/helloworld_analytic_diff.o -c /home/zpw/github/SLAM/Slambook2/ch06/CeresExamples/helloworld_analytic_diff.cc

CMakeFiles/helloworld_analytic_diff.dir/helloworld_analytic_diff.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/helloworld_analytic_diff.dir/helloworld_analytic_diff.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zpw/github/SLAM/Slambook2/ch06/CeresExamples/helloworld_analytic_diff.cc > CMakeFiles/helloworld_analytic_diff.dir/helloworld_analytic_diff.i

CMakeFiles/helloworld_analytic_diff.dir/helloworld_analytic_diff.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/helloworld_analytic_diff.dir/helloworld_analytic_diff.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zpw/github/SLAM/Slambook2/ch06/CeresExamples/helloworld_analytic_diff.cc -o CMakeFiles/helloworld_analytic_diff.dir/helloworld_analytic_diff.s

# Object files for target helloworld_analytic_diff
helloworld_analytic_diff_OBJECTS = \
"CMakeFiles/helloworld_analytic_diff.dir/helloworld_analytic_diff.o"

# External object files for target helloworld_analytic_diff
helloworld_analytic_diff_EXTERNAL_OBJECTS =

helloworld_analytic_diff: CMakeFiles/helloworld_analytic_diff.dir/helloworld_analytic_diff.o
helloworld_analytic_diff: CMakeFiles/helloworld_analytic_diff.dir/build.make
helloworld_analytic_diff: CMakeFiles/helloworld_analytic_diff.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zpw/github/SLAM/Slambook2/ch06/CeresExamples/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable helloworld_analytic_diff"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/helloworld_analytic_diff.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/helloworld_analytic_diff.dir/build: helloworld_analytic_diff

.PHONY : CMakeFiles/helloworld_analytic_diff.dir/build

CMakeFiles/helloworld_analytic_diff.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/helloworld_analytic_diff.dir/cmake_clean.cmake
.PHONY : CMakeFiles/helloworld_analytic_diff.dir/clean

CMakeFiles/helloworld_analytic_diff.dir/depend:
	cd /home/zpw/github/SLAM/Slambook2/ch06/CeresExamples/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zpw/github/SLAM/Slambook2/ch06/CeresExamples /home/zpw/github/SLAM/Slambook2/ch06/CeresExamples /home/zpw/github/SLAM/Slambook2/ch06/CeresExamples/cmake-build-debug /home/zpw/github/SLAM/Slambook2/ch06/CeresExamples/cmake-build-debug /home/zpw/github/SLAM/Slambook2/ch06/CeresExamples/cmake-build-debug/CMakeFiles/helloworld_analytic_diff.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/helloworld_analytic_diff.dir/depend
