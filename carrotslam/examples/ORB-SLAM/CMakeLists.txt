ADD_DEFINITIONS(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

SET(LINKLIBS carrotslam_core
carrotslam_nodes
carrotslam_types
pugixml
glog
)

ADD_EXECUTABLE(carrotslam_orbslam main.cpp)
TARGET_LINK_LIBRARIES(carrotslam_orbslam

${LINKLIBS}
${OpenCV_LIBS}
${Boost_LIBRARIES}
)

# 设置将项目添加到包中
if(ENABLE_SOLUTION_FOLDERS)
	set_target_properties(carrotslam_orbslam PROPERTIES FOLDER "app")
else(ENABLE_SOLUTION_FOLDERS)
	SET_TARGET_PROPERTIES(carrotslam_orbslam  PROPERTIES PROJECT_LABEL "(app) carrotslam_orbslam")
endif(ENABLE_SOLUTION_FOLDERS)
