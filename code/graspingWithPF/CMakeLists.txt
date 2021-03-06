# you can remove this line if you have an older version of CMake
cmake_minimum_required(VERSION 2.6)

SET(KEYWORD "graspingWithPF")

PROJECT(${KEYWORD})

# custom .cmake files are in the /conf subdir
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/conf)

# Find YARP.  Point the YARP_BUILD environment variable at your build.
SET(YARP_DIR "$ENV{YARP_DIR}" CACHE LOCATION "where is yarp?")

FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(ICUB REQUIRED)
FIND_PACKAGE(GSL REQUIRED)
FIND_PACKAGE(OPENCV REQUIRED)
FIND_PACKAGE(KINEMATICS REQUIRED) # C:\iCub\src\kinematics\debug\kinematics.lib ?

# Search for source code.
FILE(GLOB folder_source *.cpp *.cc *.c)
FILE(GLOB folder_header *.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${GSL_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${KINEMATICS_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${OPENCV_INCLUDE_DIR})

LINK_LIBRARIES(${GSL_LIBRARIES})
LINK_LIBRARIES(${GSLCBLAS_LIBRARIES})
LINK_LIBRARIES(${KINEMATICS_LIBRARIES})
LINK_LIBRARIES(${OPENCV_LIBRARIES})

ADD_EXECUTABLE(${KEYWORD} ${folder_source} ${folder_header})