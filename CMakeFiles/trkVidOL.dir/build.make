# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/matthias/Documents/Code/tracking-video-visualizer

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/matthias/Documents/Code/tracking-video-visualizer

# Include any dependencies generated for this target.
include CMakeFiles/trkVidOL.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/trkVidOL.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/trkVidOL.dir/flags.make

CMakeFiles/trkVidOL.dir/trkVidOL.cpp.o: CMakeFiles/trkVidOL.dir/flags.make
CMakeFiles/trkVidOL.dir/trkVidOL.cpp.o: trkVidOL.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/matthias/Documents/Code/tracking-video-visualizer/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/trkVidOL.dir/trkVidOL.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/trkVidOL.dir/trkVidOL.cpp.o -c /home/matthias/Documents/Code/tracking-video-visualizer/trkVidOL.cpp

CMakeFiles/trkVidOL.dir/trkVidOL.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/trkVidOL.dir/trkVidOL.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/matthias/Documents/Code/tracking-video-visualizer/trkVidOL.cpp > CMakeFiles/trkVidOL.dir/trkVidOL.cpp.i

CMakeFiles/trkVidOL.dir/trkVidOL.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/trkVidOL.dir/trkVidOL.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/matthias/Documents/Code/tracking-video-visualizer/trkVidOL.cpp -o CMakeFiles/trkVidOL.dir/trkVidOL.cpp.s

CMakeFiles/trkVidOL.dir/trkVidOL.cpp.o.requires:

.PHONY : CMakeFiles/trkVidOL.dir/trkVidOL.cpp.o.requires

CMakeFiles/trkVidOL.dir/trkVidOL.cpp.o.provides: CMakeFiles/trkVidOL.dir/trkVidOL.cpp.o.requires
	$(MAKE) -f CMakeFiles/trkVidOL.dir/build.make CMakeFiles/trkVidOL.dir/trkVidOL.cpp.o.provides.build
.PHONY : CMakeFiles/trkVidOL.dir/trkVidOL.cpp.o.provides

CMakeFiles/trkVidOL.dir/trkVidOL.cpp.o.provides.build: CMakeFiles/trkVidOL.dir/trkVidOL.cpp.o


# Object files for target trkVidOL
trkVidOL_OBJECTS = \
"CMakeFiles/trkVidOL.dir/trkVidOL.cpp.o"

# External object files for target trkVidOL
trkVidOL_EXTERNAL_OBJECTS =

trkVidOL: CMakeFiles/trkVidOL.dir/trkVidOL.cpp.o
trkVidOL: CMakeFiles/trkVidOL.dir/build.make
trkVidOL: /usr/local/lib/libopencv_dnn.so.4.0.0
trkVidOL: /usr/local/lib/libopencv_gapi.so.4.0.0
trkVidOL: /usr/local/lib/libopencv_ml.so.4.0.0
trkVidOL: /usr/local/lib/libopencv_objdetect.so.4.0.0
trkVidOL: /usr/local/lib/libopencv_photo.so.4.0.0
trkVidOL: /usr/local/lib/libopencv_stitching.so.4.0.0
trkVidOL: /usr/local/lib/libopencv_video.so.4.0.0
trkVidOL: /usr/local/lib/libopencv_calib3d.so.4.0.0
trkVidOL: /usr/local/lib/libopencv_features2d.so.4.0.0
trkVidOL: /usr/local/lib/libopencv_flann.so.4.0.0
trkVidOL: /usr/local/lib/libopencv_highgui.so.4.0.0
trkVidOL: /usr/local/lib/libopencv_videoio.so.4.0.0
trkVidOL: /usr/local/lib/libopencv_imgcodecs.so.4.0.0
trkVidOL: /usr/local/lib/libopencv_imgproc.so.4.0.0
trkVidOL: /usr/local/lib/libopencv_core.so.4.0.0
trkVidOL: CMakeFiles/trkVidOL.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/matthias/Documents/Code/tracking-video-visualizer/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable trkVidOL"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/trkVidOL.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/trkVidOL.dir/build: trkVidOL

.PHONY : CMakeFiles/trkVidOL.dir/build

CMakeFiles/trkVidOL.dir/requires: CMakeFiles/trkVidOL.dir/trkVidOL.cpp.o.requires

.PHONY : CMakeFiles/trkVidOL.dir/requires

CMakeFiles/trkVidOL.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/trkVidOL.dir/cmake_clean.cmake
.PHONY : CMakeFiles/trkVidOL.dir/clean

CMakeFiles/trkVidOL.dir/depend:
	cd /home/matthias/Documents/Code/tracking-video-visualizer && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/matthias/Documents/Code/tracking-video-visualizer /home/matthias/Documents/Code/tracking-video-visualizer /home/matthias/Documents/Code/tracking-video-visualizer /home/matthias/Documents/Code/tracking-video-visualizer /home/matthias/Documents/Code/tracking-video-visualizer/CMakeFiles/trkVidOL.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/trkVidOL.dir/depend

