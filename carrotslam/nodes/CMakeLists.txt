#INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/include")
ADD_LIBRARY(carrotslam_nodes
  tum_dataset_reader.cpp
  rgbd_tutorial_vo.cpp
)

