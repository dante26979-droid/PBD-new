################################################################################
# general CMake and project setup
################################################################################
cmake_minimum_required(VERSION 3.1)

project(PositionBasedDynamics)

include(CTest)

set(PROJECT_PATH ${PROJECT_SOURCE_DIR})
if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-idirafter${PROJECT_SOURCE_DIR})
else()
	include_directories(${PROJECT_SOURCE_DIR})
endif()

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMake" ${CMAKE_MODULE_PATH})
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

include(${PROJECT_PATH}/CMake/Common.cmake)

set(PBD_EXTERNAL_OPENMP_CMAKE_ARGS "")
if (USE_OpenMP AND OPENMP_FOUND)
	list(APPEND PBD_EXTERNAL_OPENMP_CMAKE_ARGS
		-DOpenMP_C_FLAGS:STRING=${OpenMP_C_FLAGS}
		-DOpenMP_CXX_FLAGS:STRING=${OpenMP_CXX_FLAGS}
		-DOpenMP_C_LIB_NAMES:STRING=${OpenMP_C_LIB_NAMES}
		-DOpenMP_CXX_LIB_NAMES:STRING=${OpenMP_CXX_LIB_NAMES}
	)
	if (OpenMP_omp_LIBRARY)
		list(APPEND PBD_EXTERNAL_OPENMP_CMAKE_ARGS -DOpenMP_omp_LIBRARY:FILEPATH=${OpenMP_omp_LIBRARY})
	endif()
	foreach(_openmp_library_name gomp mingwthrd)
		if (OpenMP_${_openmp_library_name}_LIBRARY)
			list(APPEND PBD_EXTERNAL_OPENMP_CMAKE_ARGS
				-DOpenMP_${_openmp_library_name}_LIBRARY:FILEPATH=${OpenMP_${_openmp_library_name}_LIBRARY})
		endif()
	endforeach()
endif()

option(PBD_ENABLE_HAPTICS "Build OpenHaptics device support" OFF)
set(PBD_DEVIL_ROOT "${PROJECT_SOURCE_DIR}/extern/devil" CACHE PATH "DevIL SDK root")
set(PBD_OPENHAPTICS_ROOT "C:/OpenHaptics/Developer/3.5.0" CACHE PATH "OpenHaptics SDK root")

set(PBD_DEVIL_INCLUDE_DIR "${PBD_DEVIL_ROOT}/include")
if (WIN32)
	set(PBD_DEVIL_LIBRARY_DIR "${PBD_DEVIL_ROOT}/lib/x64/Release")
	set(PBD_DEVIL_LIBRARIES
		"${PBD_DEVIL_LIBRARY_DIR}/DevIL.lib"
		"${PBD_DEVIL_LIBRARY_DIR}/ILU.lib"
	)
	set(PBD_DEVIL_RUNTIME_DLLS
		"${PBD_DEVIL_LIBRARY_DIR}/DevIL.dll"
		"${PBD_DEVIL_LIBRARY_DIR}/ILU.dll"
		"${PBD_DEVIL_LIBRARY_DIR}/ILUT.dll"
	)
else()
	find_library(PBD_DEVIL_LIBRARIES NAMES IL DevIL REQUIRED)
	set(PBD_DEVIL_RUNTIME_DLLS "")
endif()

if (NOT EXISTS "${PBD_DEVIL_INCLUDE_DIR}/IL/il.h")
	message(FATAL_ERROR "DevIL headers not found. Expected ${PBD_DEVIL_INCLUDE_DIR}/IL/il.h")
endif()

add_library(PBDDevIL INTERFACE)
target_include_directories(PBDDevIL INTERFACE "${PBD_DEVIL_INCLUDE_DIR}")
target_link_libraries(PBDDevIL INTERFACE ${PBD_DEVIL_LIBRARIES})

if (PBD_ENABLE_HAPTICS)
	set(PBD_OPENHAPTICS_INCLUDE_DIRS
		"${PBD_OPENHAPTICS_ROOT}/include"
		"${PBD_OPENHAPTICS_ROOT}/utilities/include"
	)
	set(PBD_OPENHAPTICS_LIBRARIES
		"${PBD_OPENHAPTICS_ROOT}/lib/x64/Release/hd.lib"
		"${PBD_OPENHAPTICS_ROOT}/lib/x64/Release/hl.lib"
		"${PBD_OPENHAPTICS_ROOT}/utilities/lib/x64/Release/HapticMouse.lib"
		"${PBD_OPENHAPTICS_ROOT}/utilities/lib/x64/Release/hdu.lib"
		"${PBD_OPENHAPTICS_ROOT}/utilities/lib/x64/Release/hlu.lib"
	)
	set(PBD_OPENHAPTICS_RUNTIME_DLLS
		"${PBD_OPENHAPTICS_ROOT}/lib/x64/Release/hd.dll"
		"${PBD_OPENHAPTICS_ROOT}/lib/x64/Release/hl.dll"
	)
	foreach(_openhaptics_include ${PBD_OPENHAPTICS_INCLUDE_DIRS})
		if (NOT EXISTS "${_openhaptics_include}")
			message(FATAL_ERROR "OpenHaptics include directory not found: ${_openhaptics_include}")
		endif()
	endforeach()
	foreach(_openhaptics_lib ${PBD_OPENHAPTICS_LIBRARIES})
		if (NOT EXISTS "${_openhaptics_lib}")
			message(FATAL_ERROR "OpenHaptics library not found: ${_openhaptics_lib}")
		endif()
	endforeach()
	foreach(_openhaptics_dll ${PBD_OPENHAPTICS_RUNTIME_DLLS})
		if (NOT EXISTS "${_openhaptics_dll}")
			message(FATAL_ERROR "OpenHaptics runtime DLL not found: ${_openhaptics_dll}")
		endif()
	endforeach()
	add_library(PBDOpenHaptics INTERFACE)
	target_compile_definitions(PBDOpenHaptics INTERFACE PBD_ENABLE_HAPTICS)
	target_include_directories(PBDOpenHaptics INTERFACE ${PBD_OPENHAPTICS_INCLUDE_DIRS})
	target_link_libraries(PBDOpenHaptics INTERFACE ${PBD_OPENHAPTICS_LIBRARIES})
endif()

if (NOT WIN32)
	message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
endif()

if (DEFINED PBD_EXTERNALINSTALLDIR)
	set(ExternalInstallDir "${PBD_EXTERNALINSTALLDIR}" CACHE INTERNAL "External install directory")	
else()
	set(ExternalInstallDir "${PROJECT_BINARY_DIR}" CACHE INTERNAL "External install directory")	
endif()	

include(ExternalProject)

## Eigen3 is used by most of the libraries that follow
find_package(Eigen3 REQUIRED)
if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -idirafter${EIGEN3_INCLUDE_DIR}")
endif()

set(EXT_CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE})
if (NOT ${CMAKE_BUILD_TYPE} STREQUAL "Debug")
	set(EXT_CMAKE_BUILD_TYPE "Release")
endif()

## Discregrid
if ((DEFINED Discregrid_INCLUDE_DIR) AND (DEFINED Discregrid_DEBUG_LIB) AND (DEFINED Discregrid_LIB))
	message(STATUS "Using ${Discregrid_INCLUDE_DIR}")
	set(Discregrid_LIBRARIES optimized ${Discregrid_LIB} debug ${Discregrid_DEBUG_LIB})
else()
	ExternalProject_Add(
	   Ext_Discregrid
	   PREFIX "${CMAKE_BINARY_DIR}/extern/Discregrid"
	   GIT_REPOSITORY https://github.com/InteractiveComputerGraphics/Discregrid.git
	   GIT_TAG "4c27e1cc88be828c6ac5b8a05759ac7e01cf79e9"
	   INSTALL_DIR ${ExternalInstallDir}/Discregrid
	   CMAKE_ARGS -DCMAKE_BUILD_TYPE:STRING=${EXT_CMAKE_BUILD_TYPE} -DCMAKE_INSTALL_PREFIX:PATH=${ExternalInstallDir}/Discregrid
	   -DCMAKE_POLICY_VERSION_MINIMUM:STRING=3.5
	   ${PBD_EXTERNAL_OPENMP_CMAKE_ARGS}
	   -DBUILD_CMD_EXECUTABLE:BOOL=0 -DEIGEN3_INCLUDE_DIR:PATH=${EIGEN3_INCLUDE_DIR} -DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}
	) 
	ExternalProject_Get_Property(Ext_Discregrid INSTALL_DIR)
	set(Discregrid_INCLUDE_DIR ${INSTALL_DIR}/include)
	set(Discregrid_LIBRARIES optimized ${INSTALL_DIR}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}Discregrid${CMAKE_STATIC_LIBRARY_SUFFIX} 
							 debug ${INSTALL_DIR}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}Discregrid_d${CMAKE_STATIC_LIBRARY_SUFFIX})
	unset(INSTALL_DIR)
	message(STATUS "Building ${Discregrid_INCLUDE_DIR}")
endif()

if (DEFINED GenericParameters_INCLUDE_DIR)
	message(STATUS "Using ${GenericParameters_INCLUDE_DIR}")
else()
	## GenericParameters
	ExternalProject_Add(
	   Ext_GenericParameters
	   PREFIX "${ExternalInstallDir}/GenericParameters"
	   GIT_REPOSITORY https://github.com/InteractiveComputerGraphics/GenericParameters.git
	   GIT_TAG "a4e2744eea526270cfe38b826440d09f66473316"
	   INSTALL_DIR ${ExternalInstallDir}/GenericParameters
	   CMAKE_ARGS -DCMAKE_BUILD_TYPE=${EXT_CMAKE_BUILD_TYPE} -DCMAKE_INSTALL_PREFIX:PATH=${ExternalInstallDir}/GenericParameters -DCMAKE_POLICY_VERSION_MINIMUM:STRING=3.5 -DGENERICPARAMETERS_NO_TESTS:BOOL=1
	) 
	ExternalProject_Get_Property(Ext_GenericParameters INSTALL_DIR)
	set(GenericParameters_INCLUDE_DIR ${INSTALL_DIR}/include)
	unset(INSTALL_DIR)
endif()

add_subdirectory(PositionBasedDynamics)
add_subdirectory(Simulation)
add_subdirectory(Utils)
if (NOT PBD_LIBS_ONLY)
	include(DataCopyTargets)
	add_subdirectory(extern/glfw)
	add_subdirectory(extern/imgui)
	add_subdirectory(extern/md5)
	add_subdirectory(Demos)
	if (USE_PYTHON_BINDINGS)
		add_subdirectory(extern/pybind)
		add_subdirectory(pyPBD)
	endif ()
endif()


install(DIRECTORY ./Common
DESTINATION include
FILES_MATCHING PATTERN "*.h")

install(DIRECTORY ./data
DESTINATION include
FILES_MATCHING PATTERN "*.glsl")

install(DIRECTORY ./Demos/Visualization
DESTINATION include
FILES_MATCHING PATTERN "*.h")
