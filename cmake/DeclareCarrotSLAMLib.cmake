# define_carrotslam_lib(): Declares an CarrotSLAM library target:
#-----------------------------------------------------------------------
macro(define_carrotslam_lib name)
	internal_define_carrotslam_lib(${name} 0 0 ${ARGN}) # headers_only = 0, is_metalib=0
endmacro(define_carrotslam_lib)

# define_carrotslam_lib_header_only(): Declares an CarrotSLAM headers-only library:
#-----------------------------------------------------------------------
macro(define_carrotslam_lib_header_only name)
	internal_define_carrotslam_lib(${name} 1 0 ${ARGN}) # headers_only = 1, is_metalib=0
endmacro(define_carrotslam_lib_header_only)

# define_carrotslam_metalib(): Declares an CarrotSLAM meta-lib:
#-----------------------------------------------------------------------
macro(define_carrotslam_metalib name)
	internal_define_carrotslam_lib(${name} 1 1 ${ARGN}) # headers_only = 1, is_metalib=1
endmacro(define_carrotslam_metalib)

# Implementation of both define_carrotslam_lib() and define_carrotslam_lib_headers_only():
#-----------------------------------------------------------------------------
macro(internal_define_carrotslam_lib name headers_only is_metalib)

	# Allow programmers of carrotslam carrotslam to change the default value of build_carrotslam_LIB, which is "ON" by default.
	SET(_DEFVAL "${DEFAULT_BUILD_carrotslam_${name}}")
	IF ("${_DEFVAL}" STREQUAL "")
		SET(_DEFVAL "ON")
	ENDIF ("${_DEFVAL}" STREQUAL "")

	SET(BUILD_carrotslam_${name} ${_DEFVAL} CACHE BOOL "Build the library carrotslam_${name}")
	IF(BUILD_carrotslam_${name}) 
	# --- Start of conditional build of module ---
	
	IF(NOT ${is_metalib})
		PROJECT(carrotslam_${name})
	ENDIF(NOT ${is_metalib})
	
	# There is an optional LISTS of extra sources from the caller: 
	#  "${name}_EXTRA_SRCS" and 
	#  "${name}_EXTRA_SRCS_NAME"   <--- Must NOT contain spaces!!
	#
	#  At return from this macro, there'll be defined a variable:
	#	   "${${name}_EXTRA_SRCS_NAME}_FILES"
	#   with the list of all files under that group.
	#
	#  For code simplicity, let's use the same list, just adding the default sources there:
	LIST(APPEND ${name}_EXTRA_SRCS 
		"${CMAKE_SOURCE_DIR}/carrotslam/${name}/*.cpp"
		"${CMAKE_SOURCE_DIR}/carrotslam/${name}/*.c"
		"${CMAKE_SOURCE_DIR}/carrotslam/${name}/*.cxx"
		"${CMAKE_SOURCE_DIR}/carrotslam/${name}/*.h"
		"${CMAKE_SOURCE_DIR}/carrotslam/${name}/include/${name}/*.h"
		"${CMAKE_SOURCE_DIR}/carrotslam/${name}/include/${name}/*.hpp"
		)
	LIST(APPEND ${name}_EXTRA_SRCS_NAME
		"${name}"
		"${name}"
		"${name}"
		"${name} Internal Headers"
		"${name} Public Headers"
		"${name} Public Headers"
		)
	# Only add these ones for "normal" libraries:
	IF (NOT ${headers_only})
		LIST(APPEND ${name}_EXTRA_SRCS 
			"${CMAKE_SOURCE_DIR}/carrotslam/${name}/include/${name}/link_pragmas.h"
			)
		LIST(APPEND ${name}_EXTRA_SRCS_NAME
			"DLL link macros"
			)
	ENDIF (NOT ${headers_only})

	# Collect files
	# ---------------------------------------------------------
	LIST(LENGTH ${name}_EXTRA_SRCS N_SRCS)
	LIST(LENGTH ${name}_EXTRA_SRCS_NAME N_SRCS_NAMES)
	
	IF (NOT N_SRCS EQUAL N_SRCS_NAMES)
		MESSAGE(FATAL_ERROR "Mismatch length in ${name}_EXTRA_SRCS and ${name}_EXTRA_SRCS_NAME!")
	ENDIF (NOT N_SRCS EQUAL N_SRCS_NAMES)
	
	SET(${name}_srcs "")  # ALL the files
	
	MATH(EXPR N_SRCS "${N_SRCS}-1")  # Indices are 0-based
	
	foreach(i RANGE 0 ${N_SRCS})
		# Get i'th expression & its name:
		LIST(GET ${name}_EXTRA_SRCS      ${i} FILS_EXPR)
		LIST(GET ${name}_EXTRA_SRCS_NAME ${i} FILS_GROUP_NAME)
		
		FILE(GLOB aux_list ${FILS_EXPR})
		
		SOURCE_GROUP("${FILS_GROUP_NAME} files" FILES ${aux_list})
		
		# Add to main list:
		LIST(APPEND ${name}_srcs ${aux_list})
		# All to group lists, may be used by the user upon return from this macro:
		LIST(APPEND ${FILS_GROUP_NAME}_FILES ${aux_list})
	endforeach(i)

	# Remove _LIN files when compiling under Windows, and _WIN files when compiling under Linux.
	IF(WIN32)		
		REMOVE_MATCHING_FILES_FROM_LIST(".*_LIN.cpp" ${name}_srcs)		# Win32
	ELSE(WIN32)
		REMOVE_MATCHING_FILES_FROM_LIST(".*_WIN.cpp" ${name}_srcs)		# Apple & Unix
	ENDIF(WIN32)

	# Keep a list of unit testing files, for declaring them in /unittest:
	set(lstunittests ${${name}_srcs})
	KEEP_MATCHING_FILES_FROM_LIST(".*unittest.cpp" lstunittests)
	if(NOT "${lstunittests}" STREQUAL "")
		# We have unit tests:
		get_property(_lst_lib_test GLOBAL PROPERTY "CarrotSLAM_TEST_LIBS")
		set_property(GLOBAL PROPERTY "CarrotSLAM_TEST_LIBS" ${_lst_lib_test} carrotslam_${name})
		set_property(GLOBAL PROPERTY "carrotslam_${name}_UNIT_TEST_FILES" ${lstunittests})
	endif(NOT "${lstunittests}" STREQUAL "")


	# Don't include here the unit testing code:
	REMOVE_MATCHING_FILES_FROM_LIST(".*unittest.cpp" ${name}_srcs)


	#  Define the target:
	set(all_${name}_srcs  ${${name}_srcs})
	
	# Add main lib header (may not exist in meta-carrotslam only):
	IF (EXISTS "${CMAKE_SOURCE_DIR}/carrotslam/${name}/include/${name}.h")
		set(all_${name}_srcs ${all_${name}_srcs} "${CMAKE_SOURCE_DIR}/carrotslam/${name}/include/${name}.h")
	ENDIF (EXISTS "${CMAKE_SOURCE_DIR}/carrotslam/${name}/include/${name}.h")
		
	IF (NOT ${headers_only})

		# A libray target:
		ADD_LIBRARY(carrotslam_${name}   
			${all_${name}_srcs}      # sources
			${CarrotSLAM_VERSION_RC_FILE}  # Only !="" in Win32: the .rc file with version info
			)

	ELSE(NOT ${headers_only})

		# A custom target (needs no real compiling)
		add_custom_target(carrotslam_${name} DEPENDS ${all_${name}_srcs} SOURCES ${all_${name}_srcs})

	ENDIF (NOT ${headers_only})

	# Append to list of all carrotslam_* libraries:
	if("${ALL_CarrotSLAM_LIBS}" STREQUAL "")  # first one is different to avoid an empty first list element ";carrotslam_xxx"
		SET(ALL_CarrotSLAM_LIBS "carrotslam_${name}" CACHE INTERNAL "")  # This emulates global vars
	else("${ALL_CarrotSLAM_LIBS}" STREQUAL "")
		SET(ALL_CarrotSLAM_LIBS "${ALL_CarrotSLAM_LIBS};carrotslam_${name}" CACHE INTERNAL "")  # This emulates global vars
	endif("${ALL_CarrotSLAM_LIBS}" STREQUAL "")
	
	# Include dir for this lib:
	INCLUDE_DIRECTORIES("${CarrotSLAM_SOURCE_DIR}/carrotslam/${name}/include")
	
	# Include dirs for carrotslam_XXX carrotslam:
	set(AUX_DEPS_LIST "")
	set(AUX_EXTRA_LINK_LIBS "")
	set(AUX_ALL_DEPS_BUILD 1)  # Will be set to "0" if any dependency if not built
	FOREACH(DEP ${ARGN})
		# Only for "carrotslam_XXX" carrotslam:
		IF (${DEP} MATCHES "carrotslam_")
			STRING(REGEX REPLACE "carrotslam_(.*)" "\\1" DEP_CarrotSLAM_NAME ${DEP})
			IF(NOT "${DEP_CarrotSLAM_NAME}" STREQUAL "")
				# Include dir:
				INCLUDE_DIRECTORIES("${CarrotSLAM_SOURCE_DIR}/carrotslam/${DEP_CarrotSLAM_NAME}/include")
				
				# Link "-lcarrotslam_name", only for GCC/CLang and if both THIS and the dependence are non-header-only:
				IF(NOT ${headers_only})
					IF(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang" OR CMAKE_COMPILER_IS_GNUCXX)
						get_property(_LIB_HDRONLY GLOBAL PROPERTY "${DEP}_LIB_IS_HEADERS_ONLY")
						IF(NOT _LIB_HDRONLY)
							#MESSAGE(STATUS "adding link dep: carrotslam_${name} -> ${DEP}")
							LIST(APPEND AUX_EXTRA_LINK_LIBS ${DEP}${CarrotSLAM_LINKER_LIBS_POSTFIX})
						ENDIF(NOT _LIB_HDRONLY)
					ENDIF()
				ENDIF(NOT ${headers_only})
				
				# Append to list of carrotslam_* lib dependences:
				LIST(APPEND AUX_DEPS_LIST ${DEP})
				
				# Check if all dependencies are to be build: 
				if ("${BUILD_carrotslam_${DEP_CarrotSLAM_NAME}}" STREQUAL "OFF")
					SET(AUX_ALL_DEPS_BUILD 0)
					MESSAGE(STATUS "*Warning*: Lib carrotslam_${name} cannot be built because dependency carrotslam_${DEP_CarrotSLAM_NAME} has been disabled!")
				endif ()
				
			ENDIF(NOT "${DEP_CarrotSLAM_NAME}" STREQUAL "")
		ENDIF (${DEP} MATCHES "carrotslam_")
	ENDFOREACH(DEP)
	
	# Impossible to build? 
	if (NOT AUX_ALL_DEPS_BUILD)
		MESSAGE(STATUS "*Warning* ==> Disabling compilation of lib carrotslam_${name} for missing dependencies listed above.")		
		SET(BUILD_carrotslam_${name} OFF CACHE BOOL "Build the library carrotslam_${name}" FORCE)
	endif (NOT AUX_ALL_DEPS_BUILD)
	
	
	# Emulates a global variable:
	set_property(GLOBAL PROPERTY "carrotslam_${name}_LIB_DEPS" "${AUX_DEPS_LIST}")
	set_property(GLOBAL PROPERTY "carrotslam_${name}_LIB_IS_HEADERS_ONLY" "${headers_only}")
	set_property(GLOBAL PROPERTY "carrotslam_${name}_LIB_IS_METALIB" "${is_metalib}")

	# Dependencies between projects:
	IF(NOT "${ARGN}" STREQUAL "")
		ADD_DEPENDENCIES(carrotslam_${name} ${ARGN})
	ENDIF(NOT "${ARGN}" STREQUAL "")

	IF (NOT ${headers_only})
		TARGET_LINK_LIBRARIES(carrotslam_${name} 
			${CarrotSLAMLIB_LINKER_LIBS}
			${AUX_EXTRA_LINK_LIBS}
			)
	ENDIF (NOT ${headers_only})

	if(ENABLE_SOLUTION_FOLDERS)
		set_target_properties(carrotslam_${name} PROPERTIES FOLDER "modules")
	else(ENABLE_SOLUTION_FOLDERS)
		SET_TARGET_PROPERTIES(carrotslam_${name} PROPERTIES PROJECT_LABEL "(LIB) carrotslam_${name}")
	endif(ENABLE_SOLUTION_FOLDERS)

	# Set custom name of lib + dynamic link numbering convenions in Linux:
	IF (NOT ${headers_only})
		SET_TARGET_PROPERTIES(carrotslam_${name} PROPERTIES 
			OUTPUT_NAME ${CarrotSLAM_LIB_PREFIX}carrotslam_${name} #后期添加版本号${CarrotSLAM_DLL_VERSION_POSTFIX}
			ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/"
			RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/"
			#VERSION "${CMAKE_CarrotSLAM_VERSION_NUMBER_MAJOR}.${CMAKE_CarrotSLAM_VERSION_NUMBER_MINOR}.${CMAKE_CarrotSLAM_VERSION_NUMBER_PATCH}"
			#SOVERSION ${CMAKE_CarrotSLAM_VERSION_NUMBER_MAJOR}.${CMAKE_CarrotSLAM_VERSION_NUMBER_MINOR}
			)
		
		# Set all header files as "ignored" (don't build!):
		# -----------------------------------------------------
		set(AUX_LIST_TO_IGNORE ${all_${name}_srcs})
		KEEP_MATCHING_FILES_FROM_LIST("^.*h$" AUX_LIST_TO_IGNORE)
		set_source_files_properties(${AUX_LIST_TO_IGNORE} PROPERTIES HEADER_FILE_ONLY true)
	
		INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/carrotslam/${name}/src/") # For include "${name}_precomp.h"
		IF(CarrotSLAM_ENABLE_PRECOMPILED_HDRS)
			IF (MSVC)
				# Precompiled hdrs for MSVC:
				# --------------------------------------
				STRING(TOUPPER ${name} NAMEUP)

				# The "use precomp.headr" for all the files...
				set_target_properties(carrotslam_${name}
					PROPERTIES
					COMPILE_FLAGS "/Yu${name}_precomp.h")

				# But for the file used to build the precomp. header:
				set_source_files_properties("${CMAKE_SOURCE_DIR}/carrotslam/${name}/src/${name}_precomp.cpp"
					PROPERTIES
					COMPILE_FLAGS "/Yc${name}_precomp.h")
			ENDIF (MSVC)
		
			SOURCE_GROUP("Precompiled headers" FILES 
				"${CMAKE_SOURCE_DIR}/carrotslam/${name}/src/${name}_precomp.cpp"
				"${CMAKE_SOURCE_DIR}/carrotslam/${name}/include/${name}_precomp.h"
				)	
		ENDIF(CarrotSLAM_ENABLE_PRECOMPILED_HDRS)

	ENDIF (NOT ${headers_only})

	# --- End of conditional build of module ---
	ENDIF(BUILD_carrotslam_${name}) 

endmacro(internal_define_carrotslam_lib)

