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
CMAKE_SOURCE_DIR = /home/ditrobotics/POMELO/TEL/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ditrobotics/POMELO/TEL/src/build

# Include any dependencies generated for this target.
include testing/CMakeFiles/stm32.dir/depend.make

# Include the progress variables for this target.
include testing/CMakeFiles/stm32.dir/progress.make

# Include the compile flags for this target's objects.
include testing/CMakeFiles/stm32.dir/flags.make

testing/CMakeFiles/stm32.dir/src/stm32.cpp.o: testing/CMakeFiles/stm32.dir/flags.make
testing/CMakeFiles/stm32.dir/src/stm32.cpp.o: ../testing/src/stm32.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ditrobotics/POMELO/TEL/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object testing/CMakeFiles/stm32.dir/src/stm32.cpp.o"
	cd /home/ditrobotics/POMELO/TEL/src/build/testing && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stm32.dir/src/stm32.cpp.o -c /home/ditrobotics/POMELO/TEL/src/testing/src/stm32.cpp

testing/CMakeFiles/stm32.dir/src/stm32.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stm32.dir/src/stm32.cpp.i"
	cd /home/ditrobotics/POMELO/TEL/src/build/testing && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ditrobotics/POMELO/TEL/src/testing/src/stm32.cpp > CMakeFiles/stm32.dir/src/stm32.cpp.i

testing/CMakeFiles/stm32.dir/src/stm32.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stm32.dir/src/stm32.cpp.s"
	cd /home/ditrobotics/POMELO/TEL/src/build/testing && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ditrobotics/POMELO/TEL/src/testing/src/stm32.cpp -o CMakeFiles/stm32.dir/src/stm32.cpp.s

# Object files for target stm32
stm32_OBJECTS = \
"CMakeFiles/stm32.dir/src/stm32.cpp.o"

# External object files for target stm32
stm32_EXTERNAL_OBJECTS =

devel/lib/testing/stm32: testing/CMakeFiles/stm32.dir/src/stm32.cpp.o
devel/lib/testing/stm32: testing/CMakeFiles/stm32.dir/build.make
devel/lib/testing/stm32: /opt/ros/noetic/lib/libroscpp.so
devel/lib/testing/stm32: /usr/lib/aarch64-linux-gnu/libpthread.so
devel/lib/testing/stm32: /usr/lib/aarch64-linux-gnu/libboost_chrono.so.1.71.0
devel/lib/testing/stm32: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so.1.71.0
devel/lib/testing/stm32: /opt/ros/noetic/lib/librosconsole.so
devel/lib/testing/stm32: /opt/ros/noetic/lib/librosconsole_log4cxx.so
devel/lib/testing/stm32: /opt/ros/noetic/lib/librosconsole_backend_interface.so
devel/lib/testing/stm32: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
devel/lib/testing/stm32: /usr/lib/aarch64-linux-gnu/libboost_regex.so.1.71.0
devel/lib/testing/stm32: /opt/ros/noetic/lib/libroscpp_serialization.so
devel/lib/testing/stm32: /opt/ros/noetic/lib/libxmlrpcpp.so
devel/lib/testing/stm32: /opt/ros/noetic/lib/librostime.so
devel/lib/testing/stm32: /usr/lib/aarch64-linux-gnu/libboost_date_time.so.1.71.0
devel/lib/testing/stm32: /opt/ros/noetic/lib/libcpp_common.so
devel/lib/testing/stm32: /usr/lib/aarch64-linux-gnu/libboost_system.so.1.71.0
devel/lib/testing/stm32: /usr/lib/aarch64-linux-gnu/libboost_thread.so.1.71.0
devel/lib/testing/stm32: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
devel/lib/testing/stm32: testing/CMakeFiles/stm32.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ditrobotics/POMELO/TEL/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../devel/lib/testing/stm32"
	cd /home/ditrobotics/POMELO/TEL/src/build/testing && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/stm32.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
testing/CMakeFiles/stm32.dir/build: devel/lib/testing/stm32

.PHONY : testing/CMakeFiles/stm32.dir/build

testing/CMakeFiles/stm32.dir/clean:
	cd /home/ditrobotics/POMELO/TEL/src/build/testing && $(CMAKE_COMMAND) -P CMakeFiles/stm32.dir/cmake_clean.cmake
.PHONY : testing/CMakeFiles/stm32.dir/clean

testing/CMakeFiles/stm32.dir/depend:
	cd /home/ditrobotics/POMELO/TEL/src/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ditrobotics/POMELO/TEL/src /home/ditrobotics/POMELO/TEL/src/testing /home/ditrobotics/POMELO/TEL/src/build /home/ditrobotics/POMELO/TEL/src/build/testing /home/ditrobotics/POMELO/TEL/src/build/testing/CMakeFiles/stm32.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : testing/CMakeFiles/stm32.dir/depend

