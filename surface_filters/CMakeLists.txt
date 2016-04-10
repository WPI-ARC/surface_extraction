cmake_minimum_required(VERSION 2.8.3)
project(surface_filters)

set(PCL_DIR "~/Desktop/pcl-1.8/install/usr/local/share/pcl-1.8")
set(BOOST_ROOT "~/Desktop/boost_1_60_0")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    pcl_ros
    pcl_conversions
    roscpp
    sensor_msgs
    nodelet
    dynamic_reconfigure
    surface_msgs
    surfaces
)

## System dependencies are found with CMake's conventions
find_package(Boost 1.60 REQUIRED COMPONENTS system)
find_package(PCL 1.8 REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
    cfg/MLS.cfg
    cfg/RGS.cfg
    cfg/SAC.cfg
    cfg/ChangeDetection.cfg
    cfg/BuildSurface.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES surface_filters
    CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs dynamic_reconfigure nodelet
#    DEPENDS system_lib
)

###########
## Build ##
###########
set(CMAKE_BUILD_TYPE Debug)

add_definitions(-std=c++11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -std=c++11 -Wall -Wextra -Werror")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-parameter -Wno-unknown-pragmas")

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS})

# Include header files so QTCreator will show them in its project view
add_library(surface_filters
    # Filters
    src/surface_filters/MovingLeastSquares.cpp
    src/surface_filters/RegionGrowingSegmentation.cpp
    src/surface_filters/SACSegmentAndFit.cpp
    src/surface_filters/ChangeDetection.cpp
    src/surface_filters/BuildSurface.cpp
    src/surface_filters/ExpandSurfaces.cpp
    src/surface_filters/Triangulate.cpp
    src/surface_filters/FreePointAccumulation.cpp include/surface_filters/ConcurrentPointGrouper.h)

add_dependencies(surface_filters
    ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg
    surface_msgs surfaces pcl_ros)

target_link_libraries(surface_filters
    ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} ${PCL_LIBRARIES}
    triangle)

#############
## Install ##
#############

## Mark the nodelet library for installations
install(TARGETS surface_filters
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_surface_filters.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
