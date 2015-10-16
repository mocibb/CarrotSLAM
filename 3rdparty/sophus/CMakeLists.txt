# TODO 模板文件不应该作为lib，直接通过INCLUDE_DIRECTORIES使用
SET(PROJECT_NAME sophus)
PROJECT(${PROJECT_NAME})

file(GLOB sophus_source 
${PROJECT_SOURCE_DIR}/include/sophus/*.hpp )

#  添加源码
SET(SOURCES  ${sophus_source})

# 添加头文件，是模板库，不链接lib
# INCLUDE_DIRECTORIES(include/sophus)
INCLUDE_DIRECTORIES(include)
