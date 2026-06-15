include(Visualization/CMakeLists.txt)

function(PBD_CONFIGURE_DEMO_TARGET TARGET_NAME)
	add_dependencies(${TARGET_NAME} ${SIMULATION_DEPENDENCIES})
	target_link_libraries(${TARGET_NAME} ${SIMULATION_LINK_LIBRARIES} PBDDevIL)
	if (PBD_ENABLE_HAPTICS)
		target_link_libraries(${TARGET_NAME} PBDOpenHaptics)
		foreach(_openhaptics_dll ${PBD_OPENHAPTICS_RUNTIME_DLLS})
			add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
				COMMAND ${CMAKE_COMMAND} -E copy_if_different
					"${_openhaptics_dll}"
					"$<TARGET_FILE_DIR:${TARGET_NAME}>")
		endforeach()
	endif()
	if (WIN32)
		foreach(_devil_dll ${PBD_DEVIL_RUNTIME_DLLS})
			if (EXISTS "${_devil_dll}")
				add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
					COMMAND ${CMAKE_COMMAND} -E copy_if_different
						"${_devil_dll}"
						"$<TARGET_FILE_DIR:${TARGET_NAME}>")
			endif()
		endforeach()
	endif()
endfunction()

# search all demos
set(PBD_DEMOS 
	BarDemo
	ClothDemo
	CosseratRodsDemo
	CouplingDemos
	DistanceFieldDemos
	FluidDemo
	GenericConstraintsDemos
	PositionBasedElasticRodsDemo
	RigidBodyDemos
	SceneLoaderDemo
	StiffRodsDemos
)

if (NOT PBD_LIBS_ONLY)
	foreach (_demo_name ${PBD_DEMOS})
		option(Build_${_demo_name} "Build ${_demo_name}"	ON)
		if (Build_${_demo_name})
			add_subdirectory (${CMAKE_CURRENT_SOURCE_DIR}/${_demo_name})
		endif (Build_${_demo_name})
	endforeach ()
endif()

install(DIRECTORY ./Common
DESTINATION include/Demos
FILES_MATCHING PATTERN "*.h")
