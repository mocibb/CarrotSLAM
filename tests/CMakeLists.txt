# ----------------------------------------------------------------------------
#   TESTS
# ----------------------------------------------------------------------------
PROJECT(tests)

SET(EXECUTABLE_OUTPUT_PATH "${${PROJECT_NAME}_BINARY_DIR}")

# -----------------------------
# Add the "make test" target:
# -----------------------------
ADD_CUSTOM_TARGET(test)
if(ENABLE_SOLUTION_FOLDERS)
	set_target_properties(test PROPERTIES FOLDER "unit tests")
endif(ENABLE_SOLUTION_FOLDERS)

# Tests based on Google gtest:
# -----------------------------

# Include gtest from embeded lib:
IF (NOT CMAKE_CarrotSLAM_HAS_GTEST_SYSTEM)
	INCLUDE_DIRECTORIES("${CMAKE_CarrotSLAM_GTEST_SRC_DIR}/include/")
ENDIF(NOT CMAKE_CarrotSLAM_HAS_GTEST_SYSTEM)

# If using system library, add C++ flags:
IF (NOT "${CMAKE_GTEST_CFLAGS}" STREQUAL "")
	ADD_DEFINITIONS(${CMAKE_GTEST_CFLAGS})
ENDIF (NOT "${CMAKE_GTEST_CFLAGS}" STREQUAL "")


get_property(LST_LIB_TESTS GLOBAL PROPERTY "CarrotSLAM_TEST_LIBS")
FOREACH(_TSTLIB ${LST_LIB_TESTS})
	STRING(REGEX REPLACE "carrotslam_(.*)" "carrotslam_\\1" _TSTLIB_DASH ${_TSTLIB})
	get_property(lstfiles GLOBAL PROPERTY "${_TSTLIB}_UNIT_TEST_FILES")

	#MESSAGE(STATUS "Generating tests for ${_TSTLIB} -> ${_TSTLIB_DASH}")
	#MESSAGE(STATUS "  Files: ${lstfiles}")

	# Test project:
	INCLUDE_DIRECTORIES("${CarrotSLAM_SOURCE_DIR}/tests")
	ADD_EXECUTABLE( test_${_TSTLIB} testing.h test_main.cpp ${lstfiles})

	# Add the required libraries for linking:
IF (CMAKE_CarrotSLAM_HAS_GTEST_SYSTEM)
	# System vesion:
	TARGET_LINK_LIBRARIES(test_${_TSTLIB} gtest)
ELSE (CMAKE_CarrotSLAM_HAS_GTEST_SYSTEM)
	# Own version:
	TARGET_LINK_LIBRARIES(test_${_TSTLIB} carrotslamgtest) 
	ADD_DEPENDENCIES(test_${_TSTLIB} carrotslamgtest)  # Assure the CarrotSLAM library is updated:
ENDIF (CMAKE_CarrotSLAM_HAS_GTEST_SYSTEM)

	set_target_properties(test_${_TSTLIB} PROPERTIES "COMPILE_DEFINITIONS" "CarrotSLAM_TEST_LIB=${_TSTLIB}")

	# Deps:
	
	get_property(lst_deps GLOBAL PROPERTY "${_TSTLIB_DASH}_LIB_DEPS")
	set(lst_deps ${_TSTLIB_DASH} ${lst_deps}) # The order in this list is very important for linking!
	FOREACH(_DEPLIB ${lst_deps})
		# Include dirs:
		STRING(REGEX REPLACE "carrotslam_(.*)" "\\1" _DEPLIB_DASH ${_DEPLIB})
		include_directories("${CarrotSLAM_LIBS_ROOT}/${_DEPLIB_DASH}/include")

		# Deps:
		ADD_DEPENDENCIES(test_${_TSTLIB} ${_DEPLIB})
		# Linking deps:
		get_property(_LIB_HDRONLY GLOBAL PROPERTY "${_DEPLIB}_LIB_IS_HEADERS_ONLY")
		IF(NOT _LIB_HDRONLY)
			TARGET_LINK_LIBRARIES(test_${_TSTLIB} ${_DEPLIB})
		ENDIF(NOT _LIB_HDRONLY)
	ENDFOREACH(_DEPLIB)
	
IF(CMAKE_COMPILER_IS_GNUCXX AND UNIX)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
ENDIF(CMAKE_COMPILER_IS_GNUCXX AND UNIX)
	
	# Run it:
	GET_TARGET_PROPERTY(GENERATED_EXE test_${_TSTLIB} LOCATION)
	ADD_CUSTOM_TARGET(run_tests_${_TSTLIB} COMMAND "${GENERATED_EXE}" "${CarrotSLAM_SOURCE_DIR}")
	ADD_DEPENDENCIES(run_tests_${_TSTLIB} test_${_TSTLIB})
	ADD_DEPENDENCIES(test run_tests_${_TSTLIB})

if(ENABLE_SOLUTION_FOLDERS)
	set_target_properties(test_${_TSTLIB} PROPERTIES FOLDER "unit tests")
	set_target_properties(run_tests_${_TSTLIB} PROPERTIES FOLDER "unit tests")
endif(ENABLE_SOLUTION_FOLDERS)
	
ENDFOREACH(_TSTLIB)


