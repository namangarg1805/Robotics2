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
CMAKE_SOURCE_DIR = /home/naman/catkin_ws/src/PX4-Autopilot

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/naman/catkin_ws/build/px4

# Include any dependencies generated for this target.
include src/modules/commander/Arming/ArmAuthorization/CMakeFiles/ArmAuthorization.dir/depend.make

# Include the progress variables for this target.
include src/modules/commander/Arming/ArmAuthorization/CMakeFiles/ArmAuthorization.dir/progress.make

# Include the compile flags for this target's objects.
include src/modules/commander/Arming/ArmAuthorization/CMakeFiles/ArmAuthorization.dir/flags.make

src/modules/commander/Arming/ArmAuthorization/CMakeFiles/ArmAuthorization.dir/ArmAuthorization.cpp.o: src/modules/commander/Arming/ArmAuthorization/CMakeFiles/ArmAuthorization.dir/flags.make
src/modules/commander/Arming/ArmAuthorization/CMakeFiles/ArmAuthorization.dir/ArmAuthorization.cpp.o: /home/naman/catkin_ws/src/PX4-Autopilot/src/modules/commander/Arming/ArmAuthorization/ArmAuthorization.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/naman/catkin_ws/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/modules/commander/Arming/ArmAuthorization/CMakeFiles/ArmAuthorization.dir/ArmAuthorization.cpp.o"
	cd /home/naman/catkin_ws/build/px4/src/modules/commander/Arming/ArmAuthorization && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ArmAuthorization.dir/ArmAuthorization.cpp.o -c /home/naman/catkin_ws/src/PX4-Autopilot/src/modules/commander/Arming/ArmAuthorization/ArmAuthorization.cpp

src/modules/commander/Arming/ArmAuthorization/CMakeFiles/ArmAuthorization.dir/ArmAuthorization.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ArmAuthorization.dir/ArmAuthorization.cpp.i"
	cd /home/naman/catkin_ws/build/px4/src/modules/commander/Arming/ArmAuthorization && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/naman/catkin_ws/src/PX4-Autopilot/src/modules/commander/Arming/ArmAuthorization/ArmAuthorization.cpp > CMakeFiles/ArmAuthorization.dir/ArmAuthorization.cpp.i

src/modules/commander/Arming/ArmAuthorization/CMakeFiles/ArmAuthorization.dir/ArmAuthorization.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ArmAuthorization.dir/ArmAuthorization.cpp.s"
	cd /home/naman/catkin_ws/build/px4/src/modules/commander/Arming/ArmAuthorization && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/naman/catkin_ws/src/PX4-Autopilot/src/modules/commander/Arming/ArmAuthorization/ArmAuthorization.cpp -o CMakeFiles/ArmAuthorization.dir/ArmAuthorization.cpp.s

# Object files for target ArmAuthorization
ArmAuthorization_OBJECTS = \
"CMakeFiles/ArmAuthorization.dir/ArmAuthorization.cpp.o"

# External object files for target ArmAuthorization
ArmAuthorization_EXTERNAL_OBJECTS =

/home/naman/catkin_ws/devel/.private/px4/lib/libArmAuthorization.a: src/modules/commander/Arming/ArmAuthorization/CMakeFiles/ArmAuthorization.dir/ArmAuthorization.cpp.o
/home/naman/catkin_ws/devel/.private/px4/lib/libArmAuthorization.a: src/modules/commander/Arming/ArmAuthorization/CMakeFiles/ArmAuthorization.dir/build.make
/home/naman/catkin_ws/devel/.private/px4/lib/libArmAuthorization.a: src/modules/commander/Arming/ArmAuthorization/CMakeFiles/ArmAuthorization.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/naman/catkin_ws/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library /home/naman/catkin_ws/devel/.private/px4/lib/libArmAuthorization.a"
	cd /home/naman/catkin_ws/build/px4/src/modules/commander/Arming/ArmAuthorization && $(CMAKE_COMMAND) -P CMakeFiles/ArmAuthorization.dir/cmake_clean_target.cmake
	cd /home/naman/catkin_ws/build/px4/src/modules/commander/Arming/ArmAuthorization && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ArmAuthorization.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/modules/commander/Arming/ArmAuthorization/CMakeFiles/ArmAuthorization.dir/build: /home/naman/catkin_ws/devel/.private/px4/lib/libArmAuthorization.a

.PHONY : src/modules/commander/Arming/ArmAuthorization/CMakeFiles/ArmAuthorization.dir/build

src/modules/commander/Arming/ArmAuthorization/CMakeFiles/ArmAuthorization.dir/clean:
	cd /home/naman/catkin_ws/build/px4/src/modules/commander/Arming/ArmAuthorization && $(CMAKE_COMMAND) -P CMakeFiles/ArmAuthorization.dir/cmake_clean.cmake
.PHONY : src/modules/commander/Arming/ArmAuthorization/CMakeFiles/ArmAuthorization.dir/clean

src/modules/commander/Arming/ArmAuthorization/CMakeFiles/ArmAuthorization.dir/depend:
	cd /home/naman/catkin_ws/build/px4 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/naman/catkin_ws/src/PX4-Autopilot /home/naman/catkin_ws/src/PX4-Autopilot/src/modules/commander/Arming/ArmAuthorization /home/naman/catkin_ws/build/px4 /home/naman/catkin_ws/build/px4/src/modules/commander/Arming/ArmAuthorization /home/naman/catkin_ws/build/px4/src/modules/commander/Arming/ArmAuthorization/CMakeFiles/ArmAuthorization.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/modules/commander/Arming/ArmAuthorization/CMakeFiles/ArmAuthorization.dir/depend

