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
CMAKE_SOURCE_DIR = /home/xu/slam14_learn/mouse_KF

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/xu/slam14_learn/mouse_KF/build

# Include any dependencies generated for this target.
include CMakeFiles/mouse_track_node.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/mouse_track_node.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/mouse_track_node.dir/flags.make

CMakeFiles/mouse_track_node.dir/main.cpp.o: CMakeFiles/mouse_track_node.dir/flags.make
CMakeFiles/mouse_track_node.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/xu/slam14_learn/mouse_KF/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/mouse_track_node.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mouse_track_node.dir/main.cpp.o -c /home/xu/slam14_learn/mouse_KF/main.cpp

CMakeFiles/mouse_track_node.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mouse_track_node.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/xu/slam14_learn/mouse_KF/main.cpp > CMakeFiles/mouse_track_node.dir/main.cpp.i

CMakeFiles/mouse_track_node.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mouse_track_node.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/xu/slam14_learn/mouse_KF/main.cpp -o CMakeFiles/mouse_track_node.dir/main.cpp.s

CMakeFiles/mouse_track_node.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/mouse_track_node.dir/main.cpp.o.requires

CMakeFiles/mouse_track_node.dir/main.cpp.o.provides: CMakeFiles/mouse_track_node.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/mouse_track_node.dir/build.make CMakeFiles/mouse_track_node.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/mouse_track_node.dir/main.cpp.o.provides

CMakeFiles/mouse_track_node.dir/main.cpp.o.provides.build: CMakeFiles/mouse_track_node.dir/main.cpp.o


# Object files for target mouse_track_node
mouse_track_node_OBJECTS = \
"CMakeFiles/mouse_track_node.dir/main.cpp.o"

# External object files for target mouse_track_node
mouse_track_node_EXTERNAL_OBJECTS =

mouse_track_node: CMakeFiles/mouse_track_node.dir/main.cpp.o
mouse_track_node: CMakeFiles/mouse_track_node.dir/build.make
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
mouse_track_node: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
mouse_track_node: CMakeFiles/mouse_track_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/xu/slam14_learn/mouse_KF/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable mouse_track_node"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mouse_track_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/mouse_track_node.dir/build: mouse_track_node

.PHONY : CMakeFiles/mouse_track_node.dir/build

CMakeFiles/mouse_track_node.dir/requires: CMakeFiles/mouse_track_node.dir/main.cpp.o.requires

.PHONY : CMakeFiles/mouse_track_node.dir/requires

CMakeFiles/mouse_track_node.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/mouse_track_node.dir/cmake_clean.cmake
.PHONY : CMakeFiles/mouse_track_node.dir/clean

CMakeFiles/mouse_track_node.dir/depend:
	cd /home/xu/slam14_learn/mouse_KF/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/xu/slam14_learn/mouse_KF /home/xu/slam14_learn/mouse_KF /home/xu/slam14_learn/mouse_KF/build /home/xu/slam14_learn/mouse_KF/build /home/xu/slam14_learn/mouse_KF/build/CMakeFiles/mouse_track_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/mouse_track_node.dir/depend

