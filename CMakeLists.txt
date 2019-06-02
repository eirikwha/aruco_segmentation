cmake_minimum_required(VERSION 2.8.3)
project(aruco_segmentation)

## Use C++11
add_definitions(--std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    sensor_msgs
    cv_bridge
)

find_package( OpenCV REQUIRED )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake matching_config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS
    include
#  LIBRARIES
    CATKIN_DEPENDS
    roscpp
    sensor_msgs
    cv_bridge
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

#file(GLOB SOURCES "src/aruco_segmentation/*.cpp")

## Declare a C++ library
#add_library(${PROJECT_NAME}
#            ${SOURCES}
#)

## Declare cpp executables


add_executable(
        create_marker
        src/create_arcuo_marker.cpp
)

add_executable(
        detect_marker
        src/detect_arcuo_marker.cpp
)

add_executable(
        generate_rand_markers
        src/generate_aruco_markers.cpp
)
## Specify libraries to link executable targets against


target_link_libraries(
        create_marker
#        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
)

target_link_libraries(
        generate_rand_markers
        #        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
)

target_link_libraries(
        detect_marker
        #        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
)