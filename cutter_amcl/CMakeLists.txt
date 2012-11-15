cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rosbuild_init()

# add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

rosbuild_add_library(amcl_pf 
                    src/pf/pf.c 
                    src/pf/pf_kdtree.c 
                    src/pf/pf_pdf.c 
                    src/pf/pf_vector.c
                    src/pf/eig3.c
                    src/pf/pf_draw.c)

rosbuild_add_library(amcl_map
                    src/map/map.c
                    src/map/map_cspace.cpp
                    src/map/map_range.c
                    src/map/map_store.c
                    src/map/map_draw.c)

rosbuild_add_library(amcl_sensors
                    src/sensors/amcl_sensor.cpp
                    src/sensors/amcl_odom.cpp
                    src/sensors/amcl_gps.cpp)
target_link_libraries(amcl_sensors amcl_map amcl_pf)

rosbuild_add_boost_directories()
rosbuild_add_executable(bin/cutter_amcl
                       src/cutter_amcl.cpp)
rosbuild_link_boost(bin/cutter_amcl signals)

target_link_libraries(bin/cutter_amcl amcl_sensors amcl_map amcl_pf)
