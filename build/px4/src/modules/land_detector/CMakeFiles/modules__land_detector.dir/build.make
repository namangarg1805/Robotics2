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
include src/modules/land_detector/CMakeFiles/modules__land_detector.dir/depend.make

# Include the progress variables for this target.
include src/modules/land_detector/CMakeFiles/modules__land_detector.dir/progress.make

# Include the compile flags for this target's objects.
include src/modules/land_detector/CMakeFiles/modules__land_detector.dir/flags.make

src/modules/land_detector/CMakeFiles/modules__land_detector.dir/land_detector_main.cpp.o: src/modules/land_detector/CMakeFiles/modules__land_detector.dir/flags.make
src/modules/land_detector/CMakeFiles/modules__land_detector.dir/land_detector_main.cpp.o: /home/naman/catkin_ws/src/PX4-Autopilot/src/modules/land_detector/land_detector_main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/naman/catkin_ws/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/modules/land_detector/CMakeFiles/modules__land_detector.dir/land_detector_main.cpp.o"
	cd /home/naman/catkin_ws/build/px4/src/modules/land_detector && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__land_detector.dir/land_detector_main.cpp.o -c /home/naman/catkin_ws/src/PX4-Autopilot/src/modules/land_detector/land_detector_main.cpp

src/modules/land_detector/CMakeFiles/modules__land_detector.dir/land_detector_main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__land_detector.dir/land_detector_main.cpp.i"
	cd /home/naman/catkin_ws/build/px4/src/modules/land_detector && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/naman/catkin_ws/src/PX4-Autopilot/src/modules/land_detector/land_detector_main.cpp > CMakeFiles/modules__land_detector.dir/land_detector_main.cpp.i

src/modules/land_detector/CMakeFiles/modules__land_detector.dir/land_detector_main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__land_detector.dir/land_detector_main.cpp.s"
	cd /home/naman/catkin_ws/build/px4/src/modules/land_detector && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/naman/catkin_ws/src/PX4-Autopilot/src/modules/land_detector/land_detector_main.cpp -o CMakeFiles/modules__land_detector.dir/land_detector_main.cpp.s

src/modules/land_detector/CMakeFiles/modules__land_detector.dir/LandDetector.cpp.o: src/modules/land_detector/CMakeFiles/modules__land_detector.dir/flags.make
src/modules/land_detector/CMakeFiles/modules__land_detector.dir/LandDetector.cpp.o: /home/naman/catkin_ws/src/PX4-Autopilot/src/modules/land_detector/LandDetector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/naman/catkin_ws/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/modules/land_detector/CMakeFiles/modules__land_detector.dir/LandDetector.cpp.o"
	cd /home/naman/catkin_ws/build/px4/src/modules/land_detector && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__land_detector.dir/LandDetector.cpp.o -c /home/naman/catkin_ws/src/PX4-Autopilot/src/modules/land_detector/LandDetector.cpp

src/modules/land_detector/CMakeFiles/modules__land_detector.dir/LandDetector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__land_detector.dir/LandDetector.cpp.i"
	cd /home/naman/catkin_ws/build/px4/src/modules/land_detector && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/naman/catkin_ws/src/PX4-Autopilot/src/modules/land_detector/LandDetector.cpp > CMakeFiles/modules__land_detector.dir/LandDetector.cpp.i

src/modules/land_detector/CMakeFiles/modules__land_detector.dir/LandDetector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__land_detector.dir/LandDetector.cpp.s"
	cd /home/naman/catkin_ws/build/px4/src/modules/land_detector && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/naman/catkin_ws/src/PX4-Autopilot/src/modules/land_detector/LandDetector.cpp -o CMakeFiles/modules__land_detector.dir/LandDetector.cpp.s

src/modules/land_detector/CMakeFiles/modules__land_detector.dir/MulticopterLandDetector.cpp.o: src/modules/land_detector/CMakeFiles/modules__land_detector.dir/flags.make
src/modules/land_detector/CMakeFiles/modules__land_detector.dir/MulticopterLandDetector.cpp.o: /home/naman/catkin_ws/src/PX4-Autopilot/src/modules/land_detector/MulticopterLandDetector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/naman/catkin_ws/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/modules/land_detector/CMakeFiles/modules__land_detector.dir/MulticopterLandDetector.cpp.o"
	cd /home/naman/catkin_ws/build/px4/src/modules/land_detector && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__land_detector.dir/MulticopterLandDetector.cpp.o -c /home/naman/catkin_ws/src/PX4-Autopilot/src/modules/land_detector/MulticopterLandDetector.cpp

src/modules/land_detector/CMakeFiles/modules__land_detector.dir/MulticopterLandDetector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__land_detector.dir/MulticopterLandDetector.cpp.i"
	cd /home/naman/catkin_ws/build/px4/src/modules/land_detector && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/naman/catkin_ws/src/PX4-Autopilot/src/modules/land_detector/MulticopterLandDetector.cpp > CMakeFiles/modules__land_detector.dir/MulticopterLandDetector.cpp.i

src/modules/land_detector/CMakeFiles/modules__land_detector.dir/MulticopterLandDetector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__land_detector.dir/MulticopterLandDetector.cpp.s"
	cd /home/naman/catkin_ws/build/px4/src/modules/land_detector && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/naman/catkin_ws/src/PX4-Autopilot/src/modules/land_detector/MulticopterLandDetector.cpp -o CMakeFiles/modules__land_detector.dir/MulticopterLandDetector.cpp.s

src/modules/land_detector/CMakeFiles/modules__land_detector.dir/FixedwingLandDetector.cpp.o: src/modules/land_detector/CMakeFiles/modules__land_detector.dir/flags.make
src/modules/land_detector/CMakeFiles/modules__land_detector.dir/FixedwingLandDetector.cpp.o: /home/naman/catkin_ws/src/PX4-Autopilot/src/modules/land_detector/FixedwingLandDetector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/naman/catkin_ws/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/modules/land_detector/CMakeFiles/modules__land_detector.dir/FixedwingLandDetector.cpp.o"
	cd /home/naman/catkin_ws/build/px4/src/modules/land_detector && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__land_detector.dir/FixedwingLandDetector.cpp.o -c /home/naman/catkin_ws/src/PX4-Autopilot/src/modules/land_detector/FixedwingLandDetector.cpp

src/modules/land_detector/CMakeFiles/modules__land_detector.dir/FixedwingLandDetector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__land_detector.dir/FixedwingLandDetector.cpp.i"
	cd /home/naman/catkin_ws/build/px4/src/modules/land_detector && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/naman/catkin_ws/src/PX4-Autopilot/src/modules/land_detector/FixedwingLandDetector.cpp > CMakeFiles/modules__land_detector.dir/FixedwingLandDetector.cpp.i

src/modules/land_detector/CMakeFiles/modules__land_detector.dir/FixedwingLandDetector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__land_detector.dir/FixedwingLandDetector.cpp.s"
	cd /home/naman/catkin_ws/build/px4/src/modules/land_detector && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/naman/catkin_ws/src/PX4-Autopilot/src/modules/land_detector/FixedwingLandDetector.cpp -o CMakeFiles/modules__land_detector.dir/FixedwingLandDetector.cpp.s

src/modules/land_detector/CMakeFiles/modules__land_detector.dir/VtolLandDetector.cpp.o: src/modules/land_detector/CMakeFiles/modules__land_detector.dir/flags.make
src/modules/land_detector/CMakeFiles/modules__land_detector.dir/VtolLandDetector.cpp.o: /home/naman/catkin_ws/src/PX4-Autopilot/src/modules/land_detector/VtolLandDetector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/naman/catkin_ws/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object src/modules/land_detector/CMakeFiles/modules__land_detector.dir/VtolLandDetector.cpp.o"
	cd /home/naman/catkin_ws/build/px4/src/modules/land_detector && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__land_detector.dir/VtolLandDetector.cpp.o -c /home/naman/catkin_ws/src/PX4-Autopilot/src/modules/land_detector/VtolLandDetector.cpp

src/modules/land_detector/CMakeFiles/modules__land_detector.dir/VtolLandDetector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__land_detector.dir/VtolLandDetector.cpp.i"
	cd /home/naman/catkin_ws/build/px4/src/modules/land_detector && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/naman/catkin_ws/src/PX4-Autopilot/src/modules/land_detector/VtolLandDetector.cpp > CMakeFiles/modules__land_detector.dir/VtolLandDetector.cpp.i

src/modules/land_detector/CMakeFiles/modules__land_detector.dir/VtolLandDetector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__land_detector.dir/VtolLandDetector.cpp.s"
	cd /home/naman/catkin_ws/build/px4/src/modules/land_detector && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/naman/catkin_ws/src/PX4-Autopilot/src/modules/land_detector/VtolLandDetector.cpp -o CMakeFiles/modules__land_detector.dir/VtolLandDetector.cpp.s

src/modules/land_detector/CMakeFiles/modules__land_detector.dir/RoverLandDetector.cpp.o: src/modules/land_detector/CMakeFiles/modules__land_detector.dir/flags.make
src/modules/land_detector/CMakeFiles/modules__land_detector.dir/RoverLandDetector.cpp.o: /home/naman/catkin_ws/src/PX4-Autopilot/src/modules/land_detector/RoverLandDetector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/naman/catkin_ws/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object src/modules/land_detector/CMakeFiles/modules__land_detector.dir/RoverLandDetector.cpp.o"
	cd /home/naman/catkin_ws/build/px4/src/modules/land_detector && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__land_detector.dir/RoverLandDetector.cpp.o -c /home/naman/catkin_ws/src/PX4-Autopilot/src/modules/land_detector/RoverLandDetector.cpp

src/modules/land_detector/CMakeFiles/modules__land_detector.dir/RoverLandDetector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__land_detector.dir/RoverLandDetector.cpp.i"
	cd /home/naman/catkin_ws/build/px4/src/modules/land_detector && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/naman/catkin_ws/src/PX4-Autopilot/src/modules/land_detector/RoverLandDetector.cpp > CMakeFiles/modules__land_detector.dir/RoverLandDetector.cpp.i

src/modules/land_detector/CMakeFiles/modules__land_detector.dir/RoverLandDetector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__land_detector.dir/RoverLandDetector.cpp.s"
	cd /home/naman/catkin_ws/build/px4/src/modules/land_detector && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/naman/catkin_ws/src/PX4-Autopilot/src/modules/land_detector/RoverLandDetector.cpp -o CMakeFiles/modules__land_detector.dir/RoverLandDetector.cpp.s

src/modules/land_detector/CMakeFiles/modules__land_detector.dir/AirshipLandDetector.cpp.o: src/modules/land_detector/CMakeFiles/modules__land_detector.dir/flags.make
src/modules/land_detector/CMakeFiles/modules__land_detector.dir/AirshipLandDetector.cpp.o: /home/naman/catkin_ws/src/PX4-Autopilot/src/modules/land_detector/AirshipLandDetector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/naman/catkin_ws/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object src/modules/land_detector/CMakeFiles/modules__land_detector.dir/AirshipLandDetector.cpp.o"
	cd /home/naman/catkin_ws/build/px4/src/modules/land_detector && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__land_detector.dir/AirshipLandDetector.cpp.o -c /home/naman/catkin_ws/src/PX4-Autopilot/src/modules/land_detector/AirshipLandDetector.cpp

src/modules/land_detector/CMakeFiles/modules__land_detector.dir/AirshipLandDetector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__land_detector.dir/AirshipLandDetector.cpp.i"
	cd /home/naman/catkin_ws/build/px4/src/modules/land_detector && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/naman/catkin_ws/src/PX4-Autopilot/src/modules/land_detector/AirshipLandDetector.cpp > CMakeFiles/modules__land_detector.dir/AirshipLandDetector.cpp.i

src/modules/land_detector/CMakeFiles/modules__land_detector.dir/AirshipLandDetector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__land_detector.dir/AirshipLandDetector.cpp.s"
	cd /home/naman/catkin_ws/build/px4/src/modules/land_detector && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/naman/catkin_ws/src/PX4-Autopilot/src/modules/land_detector/AirshipLandDetector.cpp -o CMakeFiles/modules__land_detector.dir/AirshipLandDetector.cpp.s

# Object files for target modules__land_detector
modules__land_detector_OBJECTS = \
"CMakeFiles/modules__land_detector.dir/land_detector_main.cpp.o" \
"CMakeFiles/modules__land_detector.dir/LandDetector.cpp.o" \
"CMakeFiles/modules__land_detector.dir/MulticopterLandDetector.cpp.o" \
"CMakeFiles/modules__land_detector.dir/FixedwingLandDetector.cpp.o" \
"CMakeFiles/modules__land_detector.dir/VtolLandDetector.cpp.o" \
"CMakeFiles/modules__land_detector.dir/RoverLandDetector.cpp.o" \
"CMakeFiles/modules__land_detector.dir/AirshipLandDetector.cpp.o"

# External object files for target modules__land_detector
modules__land_detector_EXTERNAL_OBJECTS =

/home/naman/catkin_ws/devel/.private/px4/lib/libmodules__land_detector.a: src/modules/land_detector/CMakeFiles/modules__land_detector.dir/land_detector_main.cpp.o
/home/naman/catkin_ws/devel/.private/px4/lib/libmodules__land_detector.a: src/modules/land_detector/CMakeFiles/modules__land_detector.dir/LandDetector.cpp.o
/home/naman/catkin_ws/devel/.private/px4/lib/libmodules__land_detector.a: src/modules/land_detector/CMakeFiles/modules__land_detector.dir/MulticopterLandDetector.cpp.o
/home/naman/catkin_ws/devel/.private/px4/lib/libmodules__land_detector.a: src/modules/land_detector/CMakeFiles/modules__land_detector.dir/FixedwingLandDetector.cpp.o
/home/naman/catkin_ws/devel/.private/px4/lib/libmodules__land_detector.a: src/modules/land_detector/CMakeFiles/modules__land_detector.dir/VtolLandDetector.cpp.o
/home/naman/catkin_ws/devel/.private/px4/lib/libmodules__land_detector.a: src/modules/land_detector/CMakeFiles/modules__land_detector.dir/RoverLandDetector.cpp.o
/home/naman/catkin_ws/devel/.private/px4/lib/libmodules__land_detector.a: src/modules/land_detector/CMakeFiles/modules__land_detector.dir/AirshipLandDetector.cpp.o
/home/naman/catkin_ws/devel/.private/px4/lib/libmodules__land_detector.a: src/modules/land_detector/CMakeFiles/modules__land_detector.dir/build.make
/home/naman/catkin_ws/devel/.private/px4/lib/libmodules__land_detector.a: src/modules/land_detector/CMakeFiles/modules__land_detector.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/naman/catkin_ws/build/px4/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Linking CXX static library /home/naman/catkin_ws/devel/.private/px4/lib/libmodules__land_detector.a"
	cd /home/naman/catkin_ws/build/px4/src/modules/land_detector && $(CMAKE_COMMAND) -P CMakeFiles/modules__land_detector.dir/cmake_clean_target.cmake
	cd /home/naman/catkin_ws/build/px4/src/modules/land_detector && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/modules__land_detector.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/modules/land_detector/CMakeFiles/modules__land_detector.dir/build: /home/naman/catkin_ws/devel/.private/px4/lib/libmodules__land_detector.a

.PHONY : src/modules/land_detector/CMakeFiles/modules__land_detector.dir/build

src/modules/land_detector/CMakeFiles/modules__land_detector.dir/clean:
	cd /home/naman/catkin_ws/build/px4/src/modules/land_detector && $(CMAKE_COMMAND) -P CMakeFiles/modules__land_detector.dir/cmake_clean.cmake
.PHONY : src/modules/land_detector/CMakeFiles/modules__land_detector.dir/clean

src/modules/land_detector/CMakeFiles/modules__land_detector.dir/depend:
	cd /home/naman/catkin_ws/build/px4 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/naman/catkin_ws/src/PX4-Autopilot /home/naman/catkin_ws/src/PX4-Autopilot/src/modules/land_detector /home/naman/catkin_ws/build/px4 /home/naman/catkin_ws/build/px4/src/modules/land_detector /home/naman/catkin_ws/build/px4/src/modules/land_detector/CMakeFiles/modules__land_detector.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/modules/land_detector/CMakeFiles/modules__land_detector.dir/depend

