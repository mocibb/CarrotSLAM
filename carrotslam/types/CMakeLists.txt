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

# Set include directory
INCLUDE_DIRECTORIES(
    ${CMAKE_CURRENT_SOURCE_DIR}
    /usr/include/eigen3/
    )

# ADD_SUBDIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/src)

# Add opencv package
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen REQUIRED)

add_library(pinhole_camera ${CMAKE_CURRENT_SOURCE_DIR}/pinhole_camera.cpp)

#Add include file and library
ADD_EXECUTABLE(CameraTest CameraTest.cpp)
TARGET_LINK_LIBRARIES(CameraTest pinhole_camera ${OpenCV_LIBS})
