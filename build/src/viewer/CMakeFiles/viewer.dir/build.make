# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.28

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/luis/projects/space_partition

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/luis/projects/space_partition/build

# Include any dependencies generated for this target.
include src/viewer/CMakeFiles/viewer.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include src/viewer/CMakeFiles/viewer.dir/compiler_depend.make

# Include the progress variables for this target.
include src/viewer/CMakeFiles/viewer.dir/progress.make

# Include the compile flags for this target's objects.
include src/viewer/CMakeFiles/viewer.dir/flags.make

src/viewer/CMakeFiles/viewer.dir/main.cpp.o: src/viewer/CMakeFiles/viewer.dir/flags.make
src/viewer/CMakeFiles/viewer.dir/main.cpp.o: /home/luis/projects/space_partition/src/viewer/main.cpp
src/viewer/CMakeFiles/viewer.dir/main.cpp.o: src/viewer/CMakeFiles/viewer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/luis/projects/space_partition/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/viewer/CMakeFiles/viewer.dir/main.cpp.o"
	cd /home/luis/projects/space_partition/build/src/viewer && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/viewer/CMakeFiles/viewer.dir/main.cpp.o -MF CMakeFiles/viewer.dir/main.cpp.o.d -o CMakeFiles/viewer.dir/main.cpp.o -c /home/luis/projects/space_partition/src/viewer/main.cpp

src/viewer/CMakeFiles/viewer.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/viewer.dir/main.cpp.i"
	cd /home/luis/projects/space_partition/build/src/viewer && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/luis/projects/space_partition/src/viewer/main.cpp > CMakeFiles/viewer.dir/main.cpp.i

src/viewer/CMakeFiles/viewer.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/viewer.dir/main.cpp.s"
	cd /home/luis/projects/space_partition/build/src/viewer && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/luis/projects/space_partition/src/viewer/main.cpp -o CMakeFiles/viewer.dir/main.cpp.s

src/viewer/CMakeFiles/viewer.dir/PartitionRenderer.cpp.o: src/viewer/CMakeFiles/viewer.dir/flags.make
src/viewer/CMakeFiles/viewer.dir/PartitionRenderer.cpp.o: /home/luis/projects/space_partition/src/viewer/PartitionRenderer.cpp
src/viewer/CMakeFiles/viewer.dir/PartitionRenderer.cpp.o: src/viewer/CMakeFiles/viewer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/luis/projects/space_partition/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/viewer/CMakeFiles/viewer.dir/PartitionRenderer.cpp.o"
	cd /home/luis/projects/space_partition/build/src/viewer && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/viewer/CMakeFiles/viewer.dir/PartitionRenderer.cpp.o -MF CMakeFiles/viewer.dir/PartitionRenderer.cpp.o.d -o CMakeFiles/viewer.dir/PartitionRenderer.cpp.o -c /home/luis/projects/space_partition/src/viewer/PartitionRenderer.cpp

src/viewer/CMakeFiles/viewer.dir/PartitionRenderer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/viewer.dir/PartitionRenderer.cpp.i"
	cd /home/luis/projects/space_partition/build/src/viewer && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/luis/projects/space_partition/src/viewer/PartitionRenderer.cpp > CMakeFiles/viewer.dir/PartitionRenderer.cpp.i

src/viewer/CMakeFiles/viewer.dir/PartitionRenderer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/viewer.dir/PartitionRenderer.cpp.s"
	cd /home/luis/projects/space_partition/build/src/viewer && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/luis/projects/space_partition/src/viewer/PartitionRenderer.cpp -o CMakeFiles/viewer.dir/PartitionRenderer.cpp.s

# Object files for target viewer
viewer_OBJECTS = \
"CMakeFiles/viewer.dir/main.cpp.o" \
"CMakeFiles/viewer.dir/PartitionRenderer.cpp.o"

# External object files for target viewer
viewer_EXTERNAL_OBJECTS =

src/viewer/viewer: src/viewer/CMakeFiles/viewer.dir/main.cpp.o
src/viewer/viewer: src/viewer/CMakeFiles/viewer.dir/PartitionRenderer.cpp.o
src/viewer/viewer: src/viewer/CMakeFiles/viewer.dir/build.make
src/viewer/viewer: src/octree_optimized/liboctree_optimized.a
src/viewer/viewer: src/kdtree/libkdtree.a
src/viewer/viewer: src/bsp/libbsp.a
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libpcl_apps.so
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libpcl_outofcore.so
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libpcl_people.so
src/viewer/viewer: /usr/lib/libOpenNI.so
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libOpenNI2.so
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libflann_cpp.so.1.9.2
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libGL.so
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libGLU.so
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libglut.so
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libglfw.so.3.3
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libpcl_surface.so
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libpcl_keypoints.so
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libpcl_tracking.so
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libpcl_recognition.so
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libpcl_registration.so
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libpcl_stereo.so
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libpcl_segmentation.so
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libpcl_features.so
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libpcl_filters.so
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libpcl_sample_consensus.so
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libpcl_ml.so
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libpcl_visualization.so
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libpcl_search.so
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libpcl_kdtree.so
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libpcl_io.so
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libpcl_octree.so
src/viewer/viewer: /usr/lib/gcc/x86_64-linux-gnu/13/libgomp.so
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libpthread.a
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libpng.so
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libz.so
src/viewer/viewer: /usr/lib/libOpenNI.so
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libOpenNI2.so
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-9.1.so.9.1.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libvtkInteractionImage-9.1.so.9.1.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-9.1.so.9.1.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libjsoncpp.so
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-9.1.so.9.1.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-9.1.so.9.1.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-9.1.so.9.1.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-9.1.so.9.1.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL2-9.1.so.9.1.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libvtkGUISupportQt-9.1.so.9.1.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-9.1.so.9.1.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-9.1.so.9.1.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-9.1.so.9.1.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-9.1.so.9.1.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-9.1.so.9.1.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libvtkIOCore-9.1.so.9.1.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-9.1.so.9.1.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-9.1.so.9.1.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-9.1.so.9.1.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libfreetype.so
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-9.1.so.9.1.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libvtkIOImage-9.1.so.9.1.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-9.1.so.9.1.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL2-9.1.so.9.1.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libvtkRenderingUI-9.1.so.9.1.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-9.1.so.9.1.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-9.1.so.9.1.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-9.1.so.9.1.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-9.1.so.9.1.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-9.1.so.9.1.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-9.1.so.9.1.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-9.1.so.9.1.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-9.1.so.9.1.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-9.1.so.9.1.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-9.1.so.9.1.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-9.1.so.9.1.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-9.1.so.9.1.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libvtkkissfft-9.1.so.9.1.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libGLEW.so
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libX11.so
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libQt5OpenGL.so.5.15.13
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.15.13
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.15.13
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.15.13
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-9.1.so.9.1.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libtbb.so.12.11
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libvtksys-9.1.so.9.1.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libpcl_common.so
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.83.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.83.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libboost_atomic.so.1.83.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so.1.83.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libboost_serialization.so.1.83.0
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/liblz4.so
src/viewer/viewer: /usr/lib/x86_64-linux-gnu/libqhull_r.so.8.0.2
src/viewer/viewer: src/viewer/CMakeFiles/viewer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/luis/projects/space_partition/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable viewer"
	cd /home/luis/projects/space_partition/build/src/viewer && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/viewer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/viewer/CMakeFiles/viewer.dir/build: src/viewer/viewer
.PHONY : src/viewer/CMakeFiles/viewer.dir/build

src/viewer/CMakeFiles/viewer.dir/clean:
	cd /home/luis/projects/space_partition/build/src/viewer && $(CMAKE_COMMAND) -P CMakeFiles/viewer.dir/cmake_clean.cmake
.PHONY : src/viewer/CMakeFiles/viewer.dir/clean

src/viewer/CMakeFiles/viewer.dir/depend:
	cd /home/luis/projects/space_partition/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/luis/projects/space_partition /home/luis/projects/space_partition/src/viewer /home/luis/projects/space_partition/build /home/luis/projects/space_partition/build/src/viewer /home/luis/projects/space_partition/build/src/viewer/CMakeFiles/viewer.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : src/viewer/CMakeFiles/viewer.dir/depend

