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
CMAKE_SOURCE_DIR = /home/zpw/github/SLAM/slambook2/ch02/useEigen

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zpw/github/SLAM/slambook2/ch02/useEigen/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/useEigen.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/useEigen.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/useEigen.dir/flags.make

CMakeFiles/useEigen.dir/main.cpp.o: CMakeFiles/useEigen.dir/flags.make
CMakeFiles/useEigen.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zpw/github/SLAM/slambook2/ch02/useEigen/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/useEigen.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/useEigen.dir/main.cpp.o -c /home/zpw/github/SLAM/slambook2/ch02/useEigen/main.cpp

CMakeFiles/useEigen.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/useEigen.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zpw/github/SLAM/slambook2/ch02/useEigen/main.cpp > CMakeFiles/useEigen.dir/main.cpp.i

CMakeFiles/useEigen.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/useEigen.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zpw/github/SLAM/slambook2/ch02/useEigen/main.cpp -o CMakeFiles/useEigen.dir/main.cpp.s

# Object files for target useEigen
useEigen_OBJECTS = \
"CMakeFiles/useEigen.dir/main.cpp.o"

# External object files for target useEigen
useEigen_EXTERNAL_OBJECTS =

useEigen: CMakeFiles/useEigen.dir/main.cpp.o
useEigen: CMakeFiles/useEigen.dir/build.make
useEigen: CMakeFiles/useEigen.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zpw/github/SLAM/slambook2/ch02/useEigen/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable useEigen"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/useEigen.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/useEigen.dir/build: useEigen

.PHONY : CMakeFiles/useEigen.dir/build

CMakeFiles/useEigen.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/useEigen.dir/cmake_clean.cmake
.PHONY : CMakeFiles/useEigen.dir/clean

CMakeFiles/useEigen.dir/depend:
	cd /home/zpw/github/SLAM/slambook2/ch02/useEigen/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zpw/github/SLAM/slambook2/ch02/useEigen /home/zpw/github/SLAM/slambook2/ch02/useEigen /home/zpw/github/SLAM/slambook2/ch02/useEigen/cmake-build-debug /home/zpw/github/SLAM/slambook2/ch02/useEigen/cmake-build-debug /home/zpw/github/SLAM/slambook2/ch02/useEigen/cmake-build-debug/CMakeFiles/useEigen.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/useEigen.dir/depend

