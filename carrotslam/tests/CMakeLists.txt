INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/include")
#ADD_LIBRARY(carrotslam_tests
#  tum_dataset_reader_test.cpp
#)

if(MSVC)
SET(LINKLIBS carrotslam_core
carrotslam_nodes
glog)
else(MSVC)
SET(LINKLIBS libcarrotslam_core.a
libcarrotslam_nodes.a
#libcarrotslam_core${CMAKE_DEBUG_POSTFIX}.a
#libcarrotslam_nodes${CMAKE_DEBUG_POSTFIX}.a
libpugixml.a
libglog.so
libgflags.so)
endif(MSVC)

ADD_EXECUTABLE(carrotslam_tests tum_dataset_reader_test.cpp)
TARGET_LINK_LIBRARIES(carrotslam_tests

${LINKLIBS}
${OpenCV_LIBS}
${Boost_LIBRARIES}
)
