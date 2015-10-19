ADD_EXECUTABLE(carrotslam_orbslam main.cpp)
TARGET_LINK_LIBRARIES(carrotslam_orbslam
libcarrotslam_core.a
libcarrotslam_nodes.a
libpugixml.a
libglog.so
libgflags.so
${OpenCV_LIBS}
${Boost_LIBRARIES}
)
