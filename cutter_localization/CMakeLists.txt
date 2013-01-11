cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
rosbuild_add_executable(odometry src/cutter_odometry.cpp)
rosbuild_add_executable(amcl_init src/amcl_init.cpp)

rosbuild_add_boost_directories()
rosbuild_add_executable(localization_evaluation src/localization_evaluation.cpp)
rosbuild_add_executable(amcl_to_state src/amcl_to_state.cpp)
rosbuild_add_executable(filter_test src/filter_test.cpp)
rosbuild_link_boost(localization_evaluation signals)
rosbuild_link_boost(amcl_to_state signals)
rosbuild_link_boost(filter_test signals)

####rosbuild_add_executable(localization_dgps src/localization_dgps.cpp)
#target_link_libraries(example ${PROJECT_NAME})
