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
include CMakeFiles/ObjectModel.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/ObjectModel.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/ObjectModel.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ObjectModel.dir/flags.make

CMakeFiles/ObjectModel.dir/Model_PreProsessing.cpp.o: CMakeFiles/ObjectModel.dir/flags.make
CMakeFiles/ObjectModel.dir/Model_PreProsessing.cpp.o: /home/adelelakour/CLionProjects/Eff_RANSAC/Model_PreProsessing.cpp
CMakeFiles/ObjectModel.dir/Model_PreProsessing.cpp.o: CMakeFiles/ObjectModel.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/adelelakour/CLionProjects/Eff_RANSAC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ObjectModel.dir/Model_PreProsessing.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/ObjectModel.dir/Model_PreProsessing.cpp.o -MF CMakeFiles/ObjectModel.dir/Model_PreProsessing.cpp.o.d -o CMakeFiles/ObjectModel.dir/Model_PreProsessing.cpp.o -c /home/adelelakour/CLionProjects/Eff_RANSAC/Model_PreProsessing.cpp

CMakeFiles/ObjectModel.dir/Model_PreProsessing.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/ObjectModel.dir/Model_PreProsessing.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/adelelakour/CLionProjects/Eff_RANSAC/Model_PreProsessing.cpp > CMakeFiles/ObjectModel.dir/Model_PreProsessing.cpp.i

CMakeFiles/ObjectModel.dir/Model_PreProsessing.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/ObjectModel.dir/Model_PreProsessing.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/adelelakour/CLionProjects/Eff_RANSAC/Model_PreProsessing.cpp -o CMakeFiles/ObjectModel.dir/Model_PreProsessing.cpp.s

# Object files for target ObjectModel
ObjectModel_OBJECTS = \
"CMakeFiles/ObjectModel.dir/Model_PreProsessing.cpp.o"

# External object files for target ObjectModel
ObjectModel_EXTERNAL_OBJECTS =

libObjectModel.a: CMakeFiles/ObjectModel.dir/Model_PreProsessing.cpp.o
libObjectModel.a: CMakeFiles/ObjectModel.dir/build.make
libObjectModel.a: CMakeFiles/ObjectModel.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/adelelakour/CLionProjects/Eff_RANSAC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libObjectModel.a"
	$(CMAKE_COMMAND) -P CMakeFiles/ObjectModel.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ObjectModel.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ObjectModel.dir/build: libObjectModel.a
.PHONY : CMakeFiles/ObjectModel.dir/build

CMakeFiles/ObjectModel.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ObjectModel.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ObjectModel.dir/clean

CMakeFiles/ObjectModel.dir/depend:
	cd /home/adelelakour/CLionProjects/Eff_RANSAC/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/adelelakour/CLionProjects/Eff_RANSAC /home/adelelakour/CLionProjects/Eff_RANSAC /home/adelelakour/CLionProjects/Eff_RANSAC/build /home/adelelakour/CLionProjects/Eff_RANSAC/build /home/adelelakour/CLionProjects/Eff_RANSAC/build/CMakeFiles/ObjectModel.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/ObjectModel.dir/depend

