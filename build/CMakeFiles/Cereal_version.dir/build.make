# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.27

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
CMAKE_COMMAND = /home/adelelakour/.local/lib/python3.10/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/adelelakour/.local/lib/python3.10/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/adelelakour/CLionProjects/Eff_RANSAC

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/adelelakour/CLionProjects/Eff_RANSAC/build

# Include any dependencies generated for this target.
include CMakeFiles/Cereal_version.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/Cereal_version.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/Cereal_version.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Cereal_version.dir/flags.make

CMakeFiles/Cereal_version.dir/main.cpp.o: CMakeFiles/Cereal_version.dir/flags.make
CMakeFiles/Cereal_version.dir/main.cpp.o: /home/adelelakour/CLionProjects/Eff_RANSAC/main.cpp
CMakeFiles/Cereal_version.dir/main.cpp.o: CMakeFiles/Cereal_version.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/adelelakour/CLionProjects/Eff_RANSAC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Cereal_version.dir/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Cereal_version.dir/main.cpp.o -MF CMakeFiles/Cereal_version.dir/main.cpp.o.d -o CMakeFiles/Cereal_version.dir/main.cpp.o -c /home/adelelakour/CLionProjects/Eff_RANSAC/main.cpp

CMakeFiles/Cereal_version.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/Cereal_version.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/adelelakour/CLionProjects/Eff_RANSAC/main.cpp > CMakeFiles/Cereal_version.dir/main.cpp.i

CMakeFiles/Cereal_version.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/Cereal_version.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/adelelakour/CLionProjects/Eff_RANSAC/main.cpp -o CMakeFiles/Cereal_version.dir/main.cpp.s

CMakeFiles/Cereal_version.dir/SerializeDeserialize.cpp.o: CMakeFiles/Cereal_version.dir/flags.make
CMakeFiles/Cereal_version.dir/SerializeDeserialize.cpp.o: /home/adelelakour/CLionProjects/Eff_RANSAC/SerializeDeserialize.cpp
CMakeFiles/Cereal_version.dir/SerializeDeserialize.cpp.o: CMakeFiles/Cereal_version.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/adelelakour/CLionProjects/Eff_RANSAC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/Cereal_version.dir/SerializeDeserialize.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Cereal_version.dir/SerializeDeserialize.cpp.o -MF CMakeFiles/Cereal_version.dir/SerializeDeserialize.cpp.o.d -o CMakeFiles/Cereal_version.dir/SerializeDeserialize.cpp.o -c /home/adelelakour/CLionProjects/Eff_RANSAC/SerializeDeserialize.cpp

CMakeFiles/Cereal_version.dir/SerializeDeserialize.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/Cereal_version.dir/SerializeDeserialize.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/adelelakour/CLionProjects/Eff_RANSAC/SerializeDeserialize.cpp > CMakeFiles/Cereal_version.dir/SerializeDeserialize.cpp.i

CMakeFiles/Cereal_version.dir/SerializeDeserialize.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/Cereal_version.dir/SerializeDeserialize.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/adelelakour/CLionProjects/Eff_RANSAC/SerializeDeserialize.cpp -o CMakeFiles/Cereal_version.dir/SerializeDeserialize.cpp.s

CMakeFiles/Cereal_version.dir/Database.cpp.o: CMakeFiles/Cereal_version.dir/flags.make
CMakeFiles/Cereal_version.dir/Database.cpp.o: /home/adelelakour/CLionProjects/Eff_RANSAC/Database.cpp
CMakeFiles/Cereal_version.dir/Database.cpp.o: CMakeFiles/Cereal_version.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/adelelakour/CLionProjects/Eff_RANSAC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/Cereal_version.dir/Database.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Cereal_version.dir/Database.cpp.o -MF CMakeFiles/Cereal_version.dir/Database.cpp.o.d -o CMakeFiles/Cereal_version.dir/Database.cpp.o -c /home/adelelakour/CLionProjects/Eff_RANSAC/Database.cpp

CMakeFiles/Cereal_version.dir/Database.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/Cereal_version.dir/Database.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/adelelakour/CLionProjects/Eff_RANSAC/Database.cpp > CMakeFiles/Cereal_version.dir/Database.cpp.i

CMakeFiles/Cereal_version.dir/Database.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/Cereal_version.dir/Database.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/adelelakour/CLionProjects/Eff_RANSAC/Database.cpp -o CMakeFiles/Cereal_version.dir/Database.cpp.s

# Object files for target Cereal_version
Cereal_version_OBJECTS = \
"CMakeFiles/Cereal_version.dir/main.cpp.o" \
"CMakeFiles/Cereal_version.dir/SerializeDeserialize.cpp.o" \
"CMakeFiles/Cereal_version.dir/Database.cpp.o"

# External object files for target Cereal_version
Cereal_version_EXTERNAL_OBJECTS =

Cereal_version: CMakeFiles/Cereal_version.dir/main.cpp.o
Cereal_version: CMakeFiles/Cereal_version.dir/SerializeDeserialize.cpp.o
Cereal_version: CMakeFiles/Cereal_version.dir/Database.cpp.o
Cereal_version: CMakeFiles/Cereal_version.dir/build.make
Cereal_version: libObjectModel.a
Cereal_version: /usr/local/lib/libAndreiUtils_core.a
Cereal_version: /usr/local/lib/libpcl_surface.so
Cereal_version: /usr/local/lib/libpcl_keypoints.so
Cereal_version: /usr/local/lib/libpcl_tracking.so
Cereal_version: /usr/local/lib/libpcl_recognition.so
Cereal_version: /usr/local/lib/libpcl_stereo.so
Cereal_version: /usr/local/lib/libpcl_outofcore.so
Cereal_version: /usr/local/lib/libpcl_people.so
Cereal_version: /usr/lib/x86_64-linux-gnu/libfreetype.so
Cereal_version: /usr/lib/x86_64-linux-gnu/libz.so
Cereal_version: /usr/lib/x86_64-linux-gnu/libjpeg.so
Cereal_version: /usr/lib/x86_64-linux-gnu/libpng.so
Cereal_version: /usr/lib/x86_64-linux-gnu/libtiff.so
Cereal_version: /usr/lib/x86_64-linux-gnu/libexpat.so
Cereal_version: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
Cereal_version: /usr/local/lib/libglfw3.a
Cereal_version: /usr/lib/x86_64-linux-gnu/libOpenGL.so
Cereal_version: /usr/lib/x86_64-linux-gnu/libGLX.so
Cereal_version: /usr/lib/x86_64-linux-gnu/libGLU.so
Cereal_version: /usr/local/lib/libpcl_registration.so
Cereal_version: /usr/local/lib/libpcl_segmentation.so
Cereal_version: /usr/local/lib/libpcl_features.so
Cereal_version: /usr/local/lib/libpcl_filters.so
Cereal_version: /usr/local/lib/libpcl_sample_consensus.so
Cereal_version: /usr/local/lib/libpcl_ml.so
Cereal_version: /usr/local/lib/libpcl_visualization.so
Cereal_version: /usr/local/lib/libpcl_search.so
Cereal_version: /usr/local/lib/libpcl_kdtree.so
Cereal_version: /usr/local/lib/libpcl_io.so
Cereal_version: /usr/local/lib/libpcl_octree.so
Cereal_version: /usr/lib/gcc/x86_64-linux-gnu/13/libgomp.so
Cereal_version: /usr/lib/x86_64-linux-gnu/libpthread.a
Cereal_version: /usr/lib/x86_64-linux-gnu/libpcap.so
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkInfovisCore-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkInteractionImage-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libjpeg.so
Cereal_version: /usr/lib/x86_64-linux-gnu/libpng.so
Cereal_version: /usr/lib/x86_64-linux-gnu/libtiff.so
Cereal_version: /usr/lib/x86_64-linux-gnu/libexpat.so
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkImagingFourier-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkalglib-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkImagingColor-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkIOXML-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkIOXMLParser-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkIOCore-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL2-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libfreetype.so
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL2-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkIOImage-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkCommonSystem-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtksys-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkDICOMParser-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libvtkmetaio-7.1.so.7.1p.1
Cereal_version: /usr/lib/x86_64-linux-gnu/libz.so
Cereal_version: /usr/lib/x86_64-linux-gnu/libGLEW.so
Cereal_version: /usr/lib/x86_64-linux-gnu/libSM.so
Cereal_version: /usr/lib/x86_64-linux-gnu/libICE.so
Cereal_version: /usr/lib/x86_64-linux-gnu/libX11.so
Cereal_version: /usr/lib/x86_64-linux-gnu/libXext.so
Cereal_version: /usr/lib/x86_64-linux-gnu/libXt.so
Cereal_version: /usr/local/lib/libpcl_common.so
Cereal_version: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.74.0
Cereal_version: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.74.0
Cereal_version: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so.1.74.0
Cereal_version: /usr/lib/x86_64-linux-gnu/libboost_serialization.so.1.74.0
Cereal_version: /usr/lib/x86_64-linux-gnu/libqhull_r.so.8.0.2
Cereal_version: /usr/lib/x86_64-linux-gnu/librt.a
Cereal_version: /usr/lib/x86_64-linux-gnu/libm.so
Cereal_version: CMakeFiles/Cereal_version.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/adelelakour/CLionProjects/Eff_RANSAC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable Cereal_version"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Cereal_version.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Cereal_version.dir/build: Cereal_version
.PHONY : CMakeFiles/Cereal_version.dir/build

CMakeFiles/Cereal_version.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Cereal_version.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Cereal_version.dir/clean

CMakeFiles/Cereal_version.dir/depend:
	cd /home/adelelakour/CLionProjects/Eff_RANSAC/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/adelelakour/CLionProjects/Eff_RANSAC /home/adelelakour/CLionProjects/Eff_RANSAC /home/adelelakour/CLionProjects/Eff_RANSAC/build /home/adelelakour/CLionProjects/Eff_RANSAC/build /home/adelelakour/CLionProjects/Eff_RANSAC/build/CMakeFiles/Cereal_version.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/Cereal_version.dir/depend

