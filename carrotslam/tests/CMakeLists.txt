INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/include")
#ADD_LIBRARY(carrotslam_tests
#  tum_dataset_reader_test.cpp
#)

SET(LINKLIBS carrotslam_core
carrotslam_nodes
pugixml
glog)

ADD_EXECUTABLE(carrotslam_tests tum_dataset_reader_test.cpp)
TARGET_LINK_LIBRARIES(carrotslam_tests

${LINKLIBS}
${OpenCV_LIBS}
${Boost_LIBRARIES}
)
