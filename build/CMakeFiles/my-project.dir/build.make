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
CMAKE_SOURCE_DIR = /home/fenixkz/yarp_tutorials/my_project

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/fenixkz/yarp_tutorials/my_project/build

# Include any dependencies generated for this target.
include CMakeFiles/my-project.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/my-project.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/my-project.dir/flags.make

CMakeFiles/my-project.dir/src/main.cpp.o: CMakeFiles/my-project.dir/flags.make
CMakeFiles/my-project.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/fenixkz/yarp_tutorials/my_project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/my-project.dir/src/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/my-project.dir/src/main.cpp.o -c /home/fenixkz/yarp_tutorials/my_project/src/main.cpp

CMakeFiles/my-project.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/my-project.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/fenixkz/yarp_tutorials/my_project/src/main.cpp > CMakeFiles/my-project.dir/src/main.cpp.i

CMakeFiles/my-project.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/my-project.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/fenixkz/yarp_tutorials/my_project/src/main.cpp -o CMakeFiles/my-project.dir/src/main.cpp.s

# Object files for target my-project
my__project_OBJECTS = \
"CMakeFiles/my-project.dir/src/main.cpp.o"

# External object files for target my-project
my__project_EXTERNAL_OBJECTS =

bin/my-project: CMakeFiles/my-project.dir/src/main.cpp.o
bin/my-project: CMakeFiles/my-project.dir/build.make
bin/my-project: /home/fenixkz/robotology-superbuild/build/install/lib/libYARP_init.so.3.6.0
bin/my-project: /home/fenixkz/robotology-superbuild/build/install/lib/libYARP_dev.so.3.6.0
bin/my-project: /home/fenixkz/robotology-superbuild/build/install/lib/libYARP_math.so.3.6.0
bin/my-project: /home/fenixkz/robotology-superbuild/build/install/lib/libYARP_sig.so.3.6.0
bin/my-project: /home/fenixkz/robotology-superbuild/build/install/lib/libYARP_os.so.3.6.0
bin/my-project: CMakeFiles/my-project.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/fenixkz/yarp_tutorials/my_project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable bin/my-project"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/my-project.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/my-project.dir/build: bin/my-project

.PHONY : CMakeFiles/my-project.dir/build

CMakeFiles/my-project.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/my-project.dir/cmake_clean.cmake
.PHONY : CMakeFiles/my-project.dir/clean

CMakeFiles/my-project.dir/depend:
	cd /home/fenixkz/yarp_tutorials/my_project/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/fenixkz/yarp_tutorials/my_project /home/fenixkz/yarp_tutorials/my_project /home/fenixkz/yarp_tutorials/my_project/build /home/fenixkz/yarp_tutorials/my_project/build /home/fenixkz/yarp_tutorials/my_project/build/CMakeFiles/my-project.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/my-project.dir/depend
