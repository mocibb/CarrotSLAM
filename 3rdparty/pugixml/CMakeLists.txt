project(pugixml)

# Enable C++11 long long for compilers that are capable of it
#if(NOT ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} STRLESS 3.1)
#	target_compile_features(pugixml PUBLIC cxx_long_long_type)
#endif()
INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR} )
ADD_LIBRARY(pugixml STATIC pugixml.cpp)
SET_TARGET_PROPERTIES(pugixml PROPERTIES VERSION 1.6 SOVERSION 1)

if(ENABLE_SOLUTION_FOLDERS)
	set_target_properties(pugixml PROPERTIES FOLDER "3rd party")
else(ENABLE_SOLUTION_FOLDERS)
	SET_TARGET_PROPERTIES(pugixml  PROPERTIES PROJECT_LABEL "(3rd party) pugixml")
endif(ENABLE_SOLUTION_FOLDERS)



