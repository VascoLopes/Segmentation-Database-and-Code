# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.2

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
CMAKE_SOURCE_DIR = /home/mfernandes/icarsc_Vasco/code

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/mfernandes/icarsc_Vasco/code/build

# Include any dependencies generated for this target.
include CMakeFiles/region_growing_rgb_segmentation.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/region_growing_rgb_segmentation.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/region_growing_rgb_segmentation.dir/flags.make

CMakeFiles/region_growing_rgb_segmentation.dir/code.cpp.o: CMakeFiles/region_growing_rgb_segmentation.dir/flags.make
CMakeFiles/region_growing_rgb_segmentation.dir/code.cpp.o: ../code.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/mfernandes/icarsc_Vasco/code/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/region_growing_rgb_segmentation.dir/code.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/region_growing_rgb_segmentation.dir/code.cpp.o -c /home/mfernandes/icarsc_Vasco/code/code.cpp

CMakeFiles/region_growing_rgb_segmentation.dir/code.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/region_growing_rgb_segmentation.dir/code.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/mfernandes/icarsc_Vasco/code/code.cpp > CMakeFiles/region_growing_rgb_segmentation.dir/code.cpp.i

CMakeFiles/region_growing_rgb_segmentation.dir/code.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/region_growing_rgb_segmentation.dir/code.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/mfernandes/icarsc_Vasco/code/code.cpp -o CMakeFiles/region_growing_rgb_segmentation.dir/code.cpp.s

CMakeFiles/region_growing_rgb_segmentation.dir/code.cpp.o.requires:
.PHONY : CMakeFiles/region_growing_rgb_segmentation.dir/code.cpp.o.requires

CMakeFiles/region_growing_rgb_segmentation.dir/code.cpp.o.provides: CMakeFiles/region_growing_rgb_segmentation.dir/code.cpp.o.requires
	$(MAKE) -f CMakeFiles/region_growing_rgb_segmentation.dir/build.make CMakeFiles/region_growing_rgb_segmentation.dir/code.cpp.o.provides.build
.PHONY : CMakeFiles/region_growing_rgb_segmentation.dir/code.cpp.o.provides

CMakeFiles/region_growing_rgb_segmentation.dir/code.cpp.o.provides.build: CMakeFiles/region_growing_rgb_segmentation.dir/code.cpp.o

# Object files for target region_growing_rgb_segmentation
region_growing_rgb_segmentation_OBJECTS = \
"CMakeFiles/region_growing_rgb_segmentation.dir/code.cpp.o"

# External object files for target region_growing_rgb_segmentation
region_growing_rgb_segmentation_EXTERNAL_OBJECTS =

region_growing_rgb_segmentation: CMakeFiles/region_growing_rgb_segmentation.dir/code.cpp.o
region_growing_rgb_segmentation: CMakeFiles/region_growing_rgb_segmentation.dir/build.make
region_growing_rgb_segmentation: /usr/local/lib/libboost_system.so
region_growing_rgb_segmentation: /usr/local/lib/libboost_filesystem.so
region_growing_rgb_segmentation: /usr/local/lib/libboost_thread.so
region_growing_rgb_segmentation: /usr/local/lib/libboost_date_time.so
region_growing_rgb_segmentation: /usr/local/lib/libboost_iostreams.so
region_growing_rgb_segmentation: /usr/local/lib/libboost_serialization.so
region_growing_rgb_segmentation: /usr/local/lib/libboost_chrono.so
region_growing_rgb_segmentation: /usr/lib/x86_64-linux-gnu/libpthread.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_common.so
region_growing_rgb_segmentation: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
region_growing_rgb_segmentation: /usr/local/lib/libpcl_kdtree.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_octree.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_search.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_sample_consensus.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_filters.so
region_growing_rgb_segmentation: /usr/lib/libOpenNI.so
region_growing_rgb_segmentation: /usr/lib/libOpenNI2.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_io.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_features.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_ml.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_segmentation.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_visualization.so
region_growing_rgb_segmentation: /usr/lib/x86_64-linux-gnu/libqhull.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_surface.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_registration.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_keypoints.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_tracking.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_recognition.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_stereo.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_outofcore.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_gpu_containers.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_gpu_utils.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_gpu_octree.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_gpu_features.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_gpu_kinfu_large_scale.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_gpu_kinfu.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_gpu_segmentation.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_cuda_features.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_cuda_segmentation.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_cuda_sample_consensus.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_people.so
region_growing_rgb_segmentation: /usr/local/lib/libboost_system.so
region_growing_rgb_segmentation: /usr/local/lib/libboost_filesystem.so
region_growing_rgb_segmentation: /usr/local/lib/libboost_thread.so
region_growing_rgb_segmentation: /usr/local/lib/libboost_date_time.so
region_growing_rgb_segmentation: /usr/local/lib/libboost_iostreams.so
region_growing_rgb_segmentation: /usr/local/lib/libboost_serialization.so
region_growing_rgb_segmentation: /usr/local/lib/libboost_chrono.so
region_growing_rgb_segmentation: /usr/lib/x86_64-linux-gnu/libpthread.so
region_growing_rgb_segmentation: /usr/lib/x86_64-linux-gnu/libqhull.so
region_growing_rgb_segmentation: /usr/lib/libOpenNI.so
region_growing_rgb_segmentation: /usr/lib/libOpenNI2.so
region_growing_rgb_segmentation: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
region_growing_rgb_segmentation: /usr/local/lib/libvtkFiltersVerdict-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkverdict-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkIOAMR-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkFiltersAMR-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkIOMovie-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkoggtheora-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkIOMINC-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkIOLSDyna-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkFiltersGeneric-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkFiltersFlowPaths-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkViewsInfovis-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkChartsCore-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkFiltersProgrammable-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkImagingStencil-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkDomainsChemistryOpenGL2-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkDomainsChemistry-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkIOExodus-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkGeovisCore-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkImagingStatistics-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkIOParallelXML-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkIOInfovis-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkRenderingLOD-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkIOExport-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkFiltersSelection-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkIOEnSight-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkIOPLY-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkRenderingVolumeOpenGL2-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkIOSQL-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtksqlite-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkIOImport-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkImagingMath-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkFiltersTexture-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkRenderingImage-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkViewsContext2D-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkFiltersParallelImaging-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkInteractionImage-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkIOParallel-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkjsoncpp-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkIOVideo-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkFiltersHyperTree-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkRenderingContextOpenGL2-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkImagingMorphological-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkFiltersSMP-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libpcl_common.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_kdtree.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_octree.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_search.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_sample_consensus.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_filters.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_io.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_features.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_ml.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_segmentation.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_visualization.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_surface.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_registration.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_keypoints.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_tracking.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_recognition.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_stereo.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_outofcore.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_gpu_containers.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_gpu_utils.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_gpu_octree.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_gpu_features.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_gpu_kinfu_large_scale.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_gpu_kinfu.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_gpu_segmentation.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_cuda_features.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_cuda_segmentation.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_cuda_sample_consensus.so
region_growing_rgb_segmentation: /usr/local/lib/libpcl_people.so
region_growing_rgb_segmentation: /usr/local/lib/libvtkInfovisLayout-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkproj4-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtklibxml2-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkInfovisCore-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkRenderingLabel-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkViewsCore-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkFiltersImaging-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkInteractionWidgets-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkRenderingVolume-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkFiltersHybrid-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkRenderingAnnotation-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkInteractionStyle-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkImagingColor-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkIONetCDF-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkIOXML-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkIOGeometry-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkIOXMLParser-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkexpat-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkexoIIc-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkNetCDF_cxx-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkNetCDF-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkhdf5_hl-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkhdf5-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkFiltersParallel-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkParallelCore-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkIOLegacy-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkFiltersModeling-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkRenderingContext2D-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkRenderingFreeType-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkfreetype-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkRenderingOpenGL2-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkRenderingCore-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkCommonColor-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkFiltersExtraction-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkFiltersStatistics-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkImagingFourier-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkalglib-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkFiltersGeometry-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkFiltersSources-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkImagingHybrid-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkIOImage-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkDICOMParser-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkIOCore-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkmetaio-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkpng-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtktiff-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkzlib-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkjpeg-7.0.so.1
region_growing_rgb_segmentation: /usr/lib/x86_64-linux-gnu/libSM.so
region_growing_rgb_segmentation: /usr/lib/x86_64-linux-gnu/libICE.so
region_growing_rgb_segmentation: /usr/lib/x86_64-linux-gnu/libX11.so
region_growing_rgb_segmentation: /usr/lib/x86_64-linux-gnu/libXext.so
region_growing_rgb_segmentation: /usr/lib/x86_64-linux-gnu/libXt.so
region_growing_rgb_segmentation: /usr/local/lib/libvtkglew-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkImagingGeneral-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkImagingSources-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkImagingCore-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkFiltersGeneral-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkFiltersCore-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkCommonExecutionModel-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkCommonComputationalGeometry-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkCommonDataModel-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkCommonMisc-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkCommonTransforms-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkCommonMath-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkCommonSystem-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtkCommonCore-7.0.so.1
region_growing_rgb_segmentation: /usr/local/lib/libvtksys-7.0.so.1
region_growing_rgb_segmentation: CMakeFiles/region_growing_rgb_segmentation.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable region_growing_rgb_segmentation"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/region_growing_rgb_segmentation.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/region_growing_rgb_segmentation.dir/build: region_growing_rgb_segmentation
.PHONY : CMakeFiles/region_growing_rgb_segmentation.dir/build

CMakeFiles/region_growing_rgb_segmentation.dir/requires: CMakeFiles/region_growing_rgb_segmentation.dir/code.cpp.o.requires
.PHONY : CMakeFiles/region_growing_rgb_segmentation.dir/requires

CMakeFiles/region_growing_rgb_segmentation.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/region_growing_rgb_segmentation.dir/cmake_clean.cmake
.PHONY : CMakeFiles/region_growing_rgb_segmentation.dir/clean

CMakeFiles/region_growing_rgb_segmentation.dir/depend:
	cd /home/mfernandes/icarsc_Vasco/code/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/mfernandes/icarsc_Vasco/code /home/mfernandes/icarsc_Vasco/code /home/mfernandes/icarsc_Vasco/code/build /home/mfernandes/icarsc_Vasco/code/build /home/mfernandes/icarsc_Vasco/code/build/CMakeFiles/region_growing_rgb_segmentation.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/region_growing_rgb_segmentation.dir/depend
