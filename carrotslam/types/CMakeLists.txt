INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/include")
ADD_LIBRARY(carrotslam_types
	feature.cpp
	pinhole_camera.cpp
)

