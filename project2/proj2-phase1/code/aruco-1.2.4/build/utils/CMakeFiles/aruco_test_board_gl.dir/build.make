# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /root/elec5660_onedrive/projects/ref_proj2phase1/src/aruco-1.2.4

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /root/elec5660_onedrive/projects/ref_proj2phase1/src/aruco-1.2.4/build

# Include any dependencies generated for this target.
include utils/CMakeFiles/aruco_test_board_gl.dir/depend.make

# Include the progress variables for this target.
include utils/CMakeFiles/aruco_test_board_gl.dir/progress.make

# Include the compile flags for this target's objects.
include utils/CMakeFiles/aruco_test_board_gl.dir/flags.make

utils/CMakeFiles/aruco_test_board_gl.dir/aruco_test_board_gl.cpp.o: utils/CMakeFiles/aruco_test_board_gl.dir/flags.make
utils/CMakeFiles/aruco_test_board_gl.dir/aruco_test_board_gl.cpp.o: ../utils/aruco_test_board_gl.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/root/elec5660_onedrive/projects/ref_proj2phase1/src/aruco-1.2.4/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object utils/CMakeFiles/aruco_test_board_gl.dir/aruco_test_board_gl.cpp.o"
	cd /root/elec5660_onedrive/projects/ref_proj2phase1/src/aruco-1.2.4/build/utils && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/aruco_test_board_gl.dir/aruco_test_board_gl.cpp.o -c /root/elec5660_onedrive/projects/ref_proj2phase1/src/aruco-1.2.4/utils/aruco_test_board_gl.cpp

utils/CMakeFiles/aruco_test_board_gl.dir/aruco_test_board_gl.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/aruco_test_board_gl.dir/aruco_test_board_gl.cpp.i"
	cd /root/elec5660_onedrive/projects/ref_proj2phase1/src/aruco-1.2.4/build/utils && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /root/elec5660_onedrive/projects/ref_proj2phase1/src/aruco-1.2.4/utils/aruco_test_board_gl.cpp > CMakeFiles/aruco_test_board_gl.dir/aruco_test_board_gl.cpp.i

utils/CMakeFiles/aruco_test_board_gl.dir/aruco_test_board_gl.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/aruco_test_board_gl.dir/aruco_test_board_gl.cpp.s"
	cd /root/elec5660_onedrive/projects/ref_proj2phase1/src/aruco-1.2.4/build/utils && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /root/elec5660_onedrive/projects/ref_proj2phase1/src/aruco-1.2.4/utils/aruco_test_board_gl.cpp -o CMakeFiles/aruco_test_board_gl.dir/aruco_test_board_gl.cpp.s

utils/CMakeFiles/aruco_test_board_gl.dir/aruco_test_board_gl.cpp.o.requires:

.PHONY : utils/CMakeFiles/aruco_test_board_gl.dir/aruco_test_board_gl.cpp.o.requires

utils/CMakeFiles/aruco_test_board_gl.dir/aruco_test_board_gl.cpp.o.provides: utils/CMakeFiles/aruco_test_board_gl.dir/aruco_test_board_gl.cpp.o.requires
	$(MAKE) -f utils/CMakeFiles/aruco_test_board_gl.dir/build.make utils/CMakeFiles/aruco_test_board_gl.dir/aruco_test_board_gl.cpp.o.provides.build
.PHONY : utils/CMakeFiles/aruco_test_board_gl.dir/aruco_test_board_gl.cpp.o.provides

utils/CMakeFiles/aruco_test_board_gl.dir/aruco_test_board_gl.cpp.o.provides.build: utils/CMakeFiles/aruco_test_board_gl.dir/aruco_test_board_gl.cpp.o


# Object files for target aruco_test_board_gl
aruco_test_board_gl_OBJECTS = \
"CMakeFiles/aruco_test_board_gl.dir/aruco_test_board_gl.cpp.o"

# External object files for target aruco_test_board_gl
aruco_test_board_gl_EXTERNAL_OBJECTS =

utils/aruco_test_board_gl: utils/CMakeFiles/aruco_test_board_gl.dir/aruco_test_board_gl.cpp.o
utils/aruco_test_board_gl: utils/CMakeFiles/aruco_test_board_gl.dir/build.make
utils/aruco_test_board_gl: src/libaruco.so.1.2.4
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stitching3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_superres3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videostab3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_aruco3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bgsegm3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bioinspired3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ccalib3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_cvv3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dpm3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_face3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_fuzzy3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_hdf3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_img_hash3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_line_descriptor3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_optflow3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_reg3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_rgbd3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_saliency3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stereo3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_structured_light3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_surface_matching3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_tracking3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xfeatures2d3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ximgproc3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xobjdetect3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xphoto3.so.3.3.1
utils/aruco_test_board_gl: /usr/lib/x86_64-linux-gnu/libGL.so
utils/aruco_test_board_gl: /usr/lib/x86_64-linux-gnu/libGLU.so
utils/aruco_test_board_gl: /usr/lib/x86_64-linux-gnu/libglut.so
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_photo3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_viz3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_phase_unwrapping3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_datasets3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_plot3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_text3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dnn3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ml3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_shape3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_video3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_calib3d3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_features2d3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_flann3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_highgui3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videoio3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgcodecs3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_objdetect3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgproc3.so.3.3.1
utils/aruco_test_board_gl: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so.3.3.1
utils/aruco_test_board_gl: utils/CMakeFiles/aruco_test_board_gl.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/root/elec5660_onedrive/projects/ref_proj2phase1/src/aruco-1.2.4/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable aruco_test_board_gl"
	cd /root/elec5660_onedrive/projects/ref_proj2phase1/src/aruco-1.2.4/build/utils && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/aruco_test_board_gl.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
utils/CMakeFiles/aruco_test_board_gl.dir/build: utils/aruco_test_board_gl

.PHONY : utils/CMakeFiles/aruco_test_board_gl.dir/build

utils/CMakeFiles/aruco_test_board_gl.dir/requires: utils/CMakeFiles/aruco_test_board_gl.dir/aruco_test_board_gl.cpp.o.requires

.PHONY : utils/CMakeFiles/aruco_test_board_gl.dir/requires

utils/CMakeFiles/aruco_test_board_gl.dir/clean:
	cd /root/elec5660_onedrive/projects/ref_proj2phase1/src/aruco-1.2.4/build/utils && $(CMAKE_COMMAND) -P CMakeFiles/aruco_test_board_gl.dir/cmake_clean.cmake
.PHONY : utils/CMakeFiles/aruco_test_board_gl.dir/clean

utils/CMakeFiles/aruco_test_board_gl.dir/depend:
	cd /root/elec5660_onedrive/projects/ref_proj2phase1/src/aruco-1.2.4/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /root/elec5660_onedrive/projects/ref_proj2phase1/src/aruco-1.2.4 /root/elec5660_onedrive/projects/ref_proj2phase1/src/aruco-1.2.4/utils /root/elec5660_onedrive/projects/ref_proj2phase1/src/aruco-1.2.4/build /root/elec5660_onedrive/projects/ref_proj2phase1/src/aruco-1.2.4/build/utils /root/elec5660_onedrive/projects/ref_proj2phase1/src/aruco-1.2.4/build/utils/CMakeFiles/aruco_test_board_gl.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : utils/CMakeFiles/aruco_test_board_gl.dir/depend

