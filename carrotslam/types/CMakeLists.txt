INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/include")
#ADD_LIBRARY(carrotslam_types
#  xxxx.cpp
#)
CMAKE_MINIMUM_REQUIRED (VERSION 2.8) #Set CMake Version
SET (CMAKE_CXX_COMPILER "g++") #Set Compiler

# Set binary directory
SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bin)

# Set library directory
SET(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)
# Set link directory
LINK_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/lib)

# Add opencv package
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)

# Set include directory
INCLUDE_DIRECTORIES(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/types
    ${Eigen3_INCLUDE_DIR}
    )

# ADD_SUBDIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/src)


add_library(pinhole_camera ${CMAKE_CURRENT_SOURCE_DIR}/pinhole_camera.cpp)

