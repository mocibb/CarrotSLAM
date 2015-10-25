# ----------------------------------------------------------------------------
#  根目录的CMake文件
#
#  注：CMake官网链接 http://www.cmake.org/
# ----------------------------------------------------------------------------

CMAKE_MINIMUM_REQUIRED (VERSION 2.8)#设定最小版CMake版本号

PROJECT (CarrotSLAM)#设定工程名称

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake") # 添加本地CMake脚本目录

INCLUDE(cmake/script_version_number.cmake REQUIRED)#添加项目版本号
include(cmake/FilterFileLists.cmake REQUIRED)#添加有用的宏
INCLUDE(cmake/DeclareCarrotSLAMLib.cmake REQUIRED)#自动添加lib文件
include(cmake/DeclareAppDependencies.cmake REQUIRED)#自动给出app的依赖lib
#C++编译器选项
ADD_DEFINITIONS(-std=c++11 -DGLOG_NO_ABBREVIATED_SEVERITIES)

#设置路径
SET(CarrotSLAM_LIBS_ROOT "${CMAKE_SOURCE_DIR}/carrotslam" CACHE INTERNAL "") 
SET(CarrotSLAM_SOURCE_DIR  ${CMAKE_CURRENT_SOURCE_DIR})
SET(CarrotSLAM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
SET(CarrotSLAM_BINARY_DIR  ${PROJECT_BINARY_DIR})

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE "Release")
ENDIF(NOT CMAKE_BUILD_TYPE)
set(CMAKE_DEBUG_POSTFIX  "d")#设置Debug下lib的名称后添加d，如CarrotSLAM_XXX100d.lib

#用于项目中添加筛选器，在vs下项目会好看一点
IF (CMAKE_VERSION VERSION_GREATER 2.8.0)
	SET(ENABLE_SOLUTION_FOLDERS ON CACHE BOOL "Group projects under virtual folders of the compiler IDE (e.g. VisualStudio)")
ENDIF (CMAKE_VERSION VERSION_GREATER 2.8.0)

if (ENABLE_SOLUTION_FOLDERS)
  SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)
  SET_PROPERTY(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "CMakeTargets")
endif (ENABLE_SOLUTION_FOLDERS)

#添加libs
FIND_PACKAGE(OpenCV REQUIRED) #通过CMake系统添加OpenCV
FIND_PACKAGE(Boost REQUIRED COMPONENTS filesystem system date_time)  #通过CMake系统添加Boost
FIND_PACKAGE(G2O REQUIRED) #通过CMake系统添加G2O
INCLUDE_DIRECTORIES("3rdparty/sophus/include")#设置包含当前目录
include(cmake/script_eigen.cmake REQUIRED)  #添加Eigen
include(cmake/script_gtest.cmake REQUIRED)       # Unit testing lib
#TODO 构建静态库还是动态库，目前先设置为静态库，后期更改为默认动态库


#设置include和link路径
INCLUDE_DIRECTORIES(".")#设置包含当前目录
FILE(GLOB _ALL_INCLUDE_DIRS ${CarrotSLAM_LIBS_ROOT}/*/include)  
FOREACH(_FILE ${_ALL_INCLUDE_DIRS})
  IF(IS_DIRECTORY ${_FILE})
    INCLUDE_DIRECTORIES(${_FILE})
  ENDIF()
ENDFOREACH()
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
INCLUDE_DIRECTORIES("3rdparty/pugixml")
LINK_DIRECTORIES("${CMAKE_BINARY_DIR}/lib")

IF(MSVC)
  INCLUDE_DIRECTORIES("3rdparty/glog/src/windows")
ENDIF(MSVC)

#设置路径，保存lib和bin到同一目录
SET( LIBRARY_OUTPUT_PATH ${CarrotSLAM_BINARY_DIR}/lib CACHE PATH "Output directory for libraries" )
SET( EXECUTABLE_OUTPUT_PATH ${CarrotSLAM_BINARY_DIR}/bin CACHE PATH "Output directory for applications" )

# ----------------------------------------------------------------------------
#      					处理子目录
# ----------------------------------------------------------------------------
ADD_SUBDIRECTORY(3rdparty)          # 第三方库的路径
ADD_SUBDIRECTORY(carrotslam)        # 本地C++ lib的路径

SET( BUILD_TESTING ON CACHE BOOL "Build CarrotSLAM tests")
IF( BUILD_TESTING)
	add_subdirectory(tests)  # Build my tests
ENDIF( BUILD_TESTING)
