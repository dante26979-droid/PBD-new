set(SIMULATION_LINK_LIBRARIES imgui glfw MD5 PositionBasedDynamics Simulation Utils)
set(SIMULATION_DEPENDENCIES imgui glfw MD5 PositionBasedDynamics Simulation Utils CopyPBDShaders CopyPBDModels CopyImguiFonts)

if(WIN32)
  set(SIMULATION_LINK_LIBRARIES opengl32.lib glu32.lib ${SIMULATION_LINK_LIBRARIES})
  #set(SIMULATION_DEPENDENCIES ${SIMULATION_DEPENDENCIES})
else()
  set(OpenGL_GL_PREFERENCE GLVND)
  find_package(OpenGL REQUIRED)
  include_directories( ${OPENGL_INCLUDE_DIRS})

  set(SIMULATION_LINK_LIBRARIES
    ${SIMULATION_LINK_LIBRARIES}
    ${OPENGL_LIBRARIES}
  )
endif()

############################################################
# Discregrid
############################################################
include_directories(${Discregrid_INCLUDE_DIR})
if (TARGET Ext_Discregrid)
	set(SIMULATION_DEPENDENCIES ${SIMULATION_DEPENDENCIES} Ext_Discregrid)
endif()
set(SIMULATION_LINK_LIBRARIES ${SIMULATION_LINK_LIBRARIES} ${Discregrid_LIBRARIES})

############################################################
# GenericParameters
############################################################
include_directories(${GenericParameters_INCLUDE_DIR})
if(TARGET Ext_GenericParameters)
	set(SIMULATION_DEPENDENCIES ${SIMULATION_DEPENDENCIES} Ext_GenericParameters)
endif()


add_executable(ClothCollisionDemo
	  ClothCollisionDemo.cpp

	  ../Common/LogWindow.cpp
	  ../Common/LogWindow.h
	  ../Common/Simulator_GUI_imgui.cpp
	  ../Common/Simulator_GUI_imgui.h
	  ../Common/imguiParameters.cpp
	  ../Common/imguiParameters.h
	  ../Common/DemoBase.cpp
	  ../Common/DemoBase.h
	  
	  ${VIS_FILES}          
	  ${PROJECT_PATH}/Common/Common.h

	  CMakeLists.txt
)

set_target_properties(ClothCollisionDemo PROPERTIES FOLDER "Demos")
set_target_properties(ClothCollisionDemo PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(ClothCollisionDemo PROPERTIES RELWITHDEBINFO_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
set_target_properties(ClothCollisionDemo PROPERTIES MINSIZEREL_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
PBD_CONFIGURE_DEMO_TARGET(ClothCollisionDemo)


add_executable(SpineDemo
	  DeformableCollisionDemo.cpp

	  ../Common/LogWindow.cpp
	  ../Common/LogWindow.h
	  ../Common/Simulator_GUI_imgui.cpp
	  ../Common/Simulator_GUI_imgui.h
	  ../Common/imguiParameters.cpp
	  ../Common/imguiParameters.h
	  ../Common/DemoBase.cpp
	  ../Common/DemoBase.h
	  
	  ${VIS_FILES}          
	  ${PROJECT_PATH}/Common/Common.h

	  CMakeLists.txt
)

set_target_properties(SpineDemo PROPERTIES FOLDER "Demos")
set_target_properties(SpineDemo PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(SpineDemo PROPERTIES RELWITHDEBINFO_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
set_target_properties(SpineDemo PROPERTIES MINSIZEREL_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
PBD_CONFIGURE_DEMO_TARGET(SpineDemo)


add_executable(FractureDemo
	  RigidBodyCollisionDemo.cpp

	  ../Common/LogWindow.cpp
	  ../Common/LogWindow.h
	  ../Common/Simulator_GUI_imgui.cpp
	  ../Common/Simulator_GUI_imgui.h
	  ../Common/imguiParameters.cpp
	  ../Common/imguiParameters.h
	  ../Common/DemoBase.cpp
	  ../Common/DemoBase.h
	  
	  ${VIS_FILES}          
	  ${PROJECT_PATH}/Common/Common.h

	  CMakeLists.txt
)

set_target_properties(FractureDemo PROPERTIES FOLDER "Demos")
set_target_properties(FractureDemo PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(FractureDemo PROPERTIES RELWITHDEBINFO_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
set_target_properties(FractureDemo PROPERTIES MINSIZEREL_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
PBD_CONFIGURE_DEMO_TARGET(FractureDemo)

add_executable(SDFReconstructionBenchmark
	  selfTest/SDFReconstructionBenchmark.cpp

	  ${PROJECT_PATH}/Demos/Common/SDFSurfaceReconstruction.h
	  ${PROJECT_PATH}/Common/Common.h

	  CMakeLists.txt
)

set_target_properties(SDFReconstructionBenchmark PROPERTIES FOLDER "Demos")
set_target_properties(SDFReconstructionBenchmark PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(SDFReconstructionBenchmark PROPERTIES RELWITHDEBINFO_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
set_target_properties(SDFReconstructionBenchmark PROPERTIES MINSIZEREL_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
target_include_directories(SDFReconstructionBenchmark PRIVATE ${Discregrid_INCLUDE_DIR})
target_link_libraries(SDFReconstructionBenchmark ${Discregrid_LIBRARIES})
if (TARGET Ext_Discregrid)
	add_dependencies(SDFReconstructionBenchmark Ext_Discregrid)
endif()

add_executable(SDFSurfaceReconstructionHelperTest
	  selfTest/SDFSurfaceReconstructionHelperTest.cpp

	  ${PROJECT_PATH}/Demos/Common/SDFSurfaceReconstruction.h
	  ${PROJECT_PATH}/Common/Common.h

	  CMakeLists.txt
)

set_target_properties(SDFSurfaceReconstructionHelperTest PROPERTIES FOLDER "Demos")
set_target_properties(SDFSurfaceReconstructionHelperTest PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(SDFSurfaceReconstructionHelperTest PROPERTIES RELWITHDEBINFO_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
set_target_properties(SDFSurfaceReconstructionHelperTest PROPERTIES MINSIZEREL_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
target_link_libraries(SDFSurfaceReconstructionHelperTest Simulation Utils)

add_executable(DemoFlowSummaryTest
	  selfTest/DemoFlowSummaryTest.cpp

	  ${PROJECT_PATH}/Demos/Common/DemoFlowSummary.h
	  ${PROJECT_PATH}/Common/Common.h

	  CMakeLists.txt
)

set_target_properties(DemoFlowSummaryTest PROPERTIES FOLDER "Demos")
set_target_properties(DemoFlowSummaryTest PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(DemoFlowSummaryTest PROPERTIES RELWITHDEBINFO_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
set_target_properties(DemoFlowSummaryTest PROPERTIES MINSIZEREL_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})

add_executable(DemoGuiStartupPolicyTest
	  selfTest/DemoGuiStartupPolicyTest.cpp

	  ${PROJECT_PATH}/Demos/Common/DemoGuiStartupPolicy.h
	  ${PROJECT_PATH}/Common/Common.h

	  CMakeLists.txt
)

set_target_properties(DemoGuiStartupPolicyTest PROPERTIES FOLDER "Demos")
set_target_properties(DemoGuiStartupPolicyTest PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(DemoGuiStartupPolicyTest PROPERTIES RELWITHDEBINFO_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
set_target_properties(DemoGuiStartupPolicyTest PROPERTIES MINSIZEREL_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})

add_executable(LegacyToolPathGateTest
	  selfTest/LegacyToolPathGateTest.cpp

	  ${PROJECT_PATH}/Demos/Common/LegacyToolPathGate.h
	  ${PROJECT_PATH}/Common/Common.h

	  CMakeLists.txt
)

set_target_properties(LegacyToolPathGateTest PROPERTIES FOLDER "Demos")
set_target_properties(LegacyToolPathGateTest PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(LegacyToolPathGateTest PROPERTIES RELWITHDEBINFO_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
set_target_properties(LegacyToolPathGateTest PROPERTIES MINSIZEREL_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})

add_executable(ToolDraggedFragmentSelectionTest
	  selfTest/ToolDraggedFragmentSelectionTest.cpp

	  ${PROJECT_PATH}/Demos/Common/ToolDraggedFragmentSelection.h
	  ${PROJECT_PATH}/Common/Common.h

	  CMakeLists.txt
)

set_target_properties(ToolDraggedFragmentSelectionTest PROPERTIES FOLDER "Demos")
set_target_properties(ToolDraggedFragmentSelectionTest PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(ToolDraggedFragmentSelectionTest PROPERTIES RELWITHDEBINFO_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
set_target_properties(ToolDraggedFragmentSelectionTest PROPERTIES MINSIZEREL_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
target_link_libraries(ToolDraggedFragmentSelectionTest Simulation Utils)

add_executable(RigidBodyMeshUpdateTest
	  selfTest/RigidBodyMeshUpdateTest.cpp

	  ${PROJECT_PATH}/Demos/Common/RigidBodyMeshUpdate.h
	  ${PROJECT_PATH}/Common/Common.h

	  CMakeLists.txt
)

set_target_properties(RigidBodyMeshUpdateTest PROPERTIES FOLDER "Demos")
set_target_properties(RigidBodyMeshUpdateTest PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(RigidBodyMeshUpdateTest PROPERTIES RELWITHDEBINFO_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
set_target_properties(RigidBodyMeshUpdateTest PROPERTIES MINSIZEREL_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
target_link_libraries(RigidBodyMeshUpdateTest Simulation Utils)

add_executable(FindEdgeKeyTest
	  selfTest/FindEdgeKeyTest.cpp

	  ${PROJECT_PATH}/Simulation/Find.h
	  ${PROJECT_PATH}/Common/Common.h

	  CMakeLists.txt
)

set_target_properties(FindEdgeKeyTest PROPERTIES FOLDER "Demos")
set_target_properties(FindEdgeKeyTest PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(FindEdgeKeyTest PROPERTIES RELWITHDEBINFO_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
set_target_properties(FindEdgeKeyTest PROPERTIES MINSIZEREL_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
target_link_libraries(FindEdgeKeyTest Simulation Utils)

add_executable(ToolCutInteractionTest
	  selfTest/ToolCutInteractionTest.cpp

	  ${PROJECT_PATH}/Simulation/Find.h
	  ${PROJECT_PATH}/Simulation/ToolCutInteraction.h
	  ${PROJECT_PATH}/Common/Common.h

	  CMakeLists.txt
)

set_target_properties(ToolCutInteractionTest PROPERTIES FOLDER "Demos")
set_target_properties(ToolCutInteractionTest PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(ToolCutInteractionTest PROPERTIES RELWITHDEBINFO_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
set_target_properties(ToolCutInteractionTest PROPERTIES MINSIZEREL_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
target_link_libraries(ToolCutInteractionTest Simulation Utils)

add_executable(ToolCutDiagnosticsTest
	  selfTest/ToolCutDiagnosticsTest.cpp

	  ${PROJECT_PATH}/Demos/Common/ToolCutDiagnostics.h
	  ${PROJECT_PATH}/Common/Common.h

	  CMakeLists.txt
)

set_target_properties(ToolCutDiagnosticsTest PROPERTIES FOLDER "Demos")
set_target_properties(ToolCutDiagnosticsTest PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(ToolCutDiagnosticsTest PROPERTIES RELWITHDEBINFO_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
set_target_properties(ToolCutDiagnosticsTest PROPERTIES MINSIZEREL_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
target_link_libraries(ToolCutDiagnosticsTest Simulation Utils)

add_executable(TextureAssetDiagnosticsTest
	  selfTest/TextureAssetDiagnosticsTest.cpp

	  ${PROJECT_PATH}/Demos/Visualization/TextureAssetDiagnostics.h
	  ${PROJECT_PATH}/Common/Common.h

	  CMakeLists.txt
)

set_target_properties(TextureAssetDiagnosticsTest PROPERTIES FOLDER "Demos")
set_target_properties(TextureAssetDiagnosticsTest PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(TextureAssetDiagnosticsTest PROPERTIES RELWITHDEBINFO_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
set_target_properties(TextureAssetDiagnosticsTest PROPERTIES MINSIZEREL_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
target_link_libraries(TextureAssetDiagnosticsTest Utils)

add_executable(LiveHapticToolControlTest
	  selfTest/LiveHapticToolControlTest.cpp

	  ${PROJECT_PATH}/Demos/Common/LiveHapticToolControl.h
	  ${PROJECT_PATH}/Common/Common.h

	  CMakeLists.txt
)

set_target_properties(LiveHapticToolControlTest PROPERTIES FOLDER "Demos")
set_target_properties(LiveHapticToolControlTest PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(LiveHapticToolControlTest PROPERTIES RELWITHDEBINFO_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
set_target_properties(LiveHapticToolControlTest PROPERTIES MINSIZEREL_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})

add_executable(WorkflowSceneTextureMappingTest
	  selfTest/WorkflowSceneTextureMappingTest.cpp

	  WorkflowSceneTextureMapping.h
	  ${PROJECT_PATH}/Common/Common.h

	  CMakeLists.txt
)

set_target_properties(WorkflowSceneTextureMappingTest PROPERTIES FOLDER "Demos")
set_target_properties(WorkflowSceneTextureMappingTest PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(WorkflowSceneTextureMappingTest PROPERTIES RELWITHDEBINFO_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
set_target_properties(WorkflowSceneTextureMappingTest PROPERTIES MINSIZEREL_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})

add_executable(WorkflowRuntimePolicyTest
	  selfTest/WorkflowRuntimePolicyTest.cpp

	  WorkflowRuntimePolicy.h
	  ${PROJECT_PATH}/Common/Common.h

	  CMakeLists.txt
)

set_target_properties(WorkflowRuntimePolicyTest PROPERTIES FOLDER "Demos")
set_target_properties(WorkflowRuntimePolicyTest PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(WorkflowRuntimePolicyTest PROPERTIES RELWITHDEBINFO_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
set_target_properties(WorkflowRuntimePolicyTest PROPERTIES MINSIZEREL_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})

add_executable(WorkflowSceneLayerStyleTest
	  selfTest/WorkflowSceneLayerStyleTest.cpp

	  WorkflowSceneLayerStyle.h
	  ${PROJECT_PATH}/Common/Common.h

	  CMakeLists.txt
)

set_target_properties(WorkflowSceneLayerStyleTest PROPERTIES FOLDER "Demos")
set_target_properties(WorkflowSceneLayerStyleTest PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(WorkflowSceneLayerStyleTest PROPERTIES RELWITHDEBINFO_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
set_target_properties(WorkflowSceneLayerStyleTest PROPERTIES MINSIZEREL_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})

add_executable(WorkflowKeyboardControlsTest
	  selfTest/WorkflowKeyboardControlsTest.cpp

	  WorkflowKeyboardControls.h
	  ${PROJECT_PATH}/Common/Common.h

	  CMakeLists.txt
)

set_target_properties(WorkflowKeyboardControlsTest PROPERTIES FOLDER "Demos")
set_target_properties(WorkflowKeyboardControlsTest PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(WorkflowKeyboardControlsTest PROPERTIES RELWITHDEBINFO_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
set_target_properties(WorkflowKeyboardControlsTest PROPERTIES MINSIZEREL_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
target_link_libraries(WorkflowKeyboardControlsTest glfw)

add_executable(ToolTargetInteractionBenchmark
	  selfTest/ToolTargetInteractionBenchmark.cpp

	  ${PROJECT_PATH}/Common/Common.h

	  CMakeLists.txt
)

set_target_properties(ToolTargetInteractionBenchmark PROPERTIES FOLDER "Demos")
set_target_properties(ToolTargetInteractionBenchmark PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(ToolTargetInteractionBenchmark PROPERTIES RELWITHDEBINFO_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
set_target_properties(ToolTargetInteractionBenchmark PROPERTIES MINSIZEREL_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
target_link_libraries(ToolTargetInteractionBenchmark Simulation Utils)

add_executable(TetModelPipelineBenchmark
	  selfTest/TetModelPipelineBenchmark.cpp

	  ${PROJECT_PATH}/Common/Common.h

	  CMakeLists.txt
)

set_target_properties(TetModelPipelineBenchmark PROPERTIES FOLDER "Demos")
set_target_properties(TetModelPipelineBenchmark PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(TetModelPipelineBenchmark PROPERTIES RELWITHDEBINFO_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
set_target_properties(TetModelPipelineBenchmark PROPERTIES MINSIZEREL_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
target_link_libraries(TetModelPipelineBenchmark Simulation Utils)

add_executable(TetModelSutureBenchmark
	  selfTest/TetModelSutureBenchmark.cpp

	  ${PROJECT_PATH}/Common/Common.h

	  CMakeLists.txt
)

set_target_properties(TetModelSutureBenchmark PROPERTIES FOLDER "Demos")
set_target_properties(TetModelSutureBenchmark PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(TetModelSutureBenchmark PROPERTIES RELWITHDEBINFO_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
set_target_properties(TetModelSutureBenchmark PROPERTIES MINSIZEREL_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
target_link_libraries(TetModelSutureBenchmark Simulation Utils)

add_executable(SutureDemo
	  SutureDemo.cpp

	  ${PROJECT_PATH}/Common/Common.h

	  CMakeLists.txt
)

set_target_properties(SutureDemo PROPERTIES FOLDER "Demos")
set_target_properties(SutureDemo PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(SutureDemo PROPERTIES RELWITHDEBINFO_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
set_target_properties(SutureDemo PROPERTIES MINSIZEREL_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
target_link_libraries(SutureDemo Simulation Utils)

add_executable(SutureVisualDemo
	  SutureVisualDemo.cpp

	  ../Common/LogWindow.cpp
	  ../Common/LogWindow.h
	  ../Common/Simulator_GUI_imgui.cpp
	  ../Common/Simulator_GUI_imgui.h
	  ../Common/imguiParameters.cpp
	  ../Common/imguiParameters.h
	  ../Common/DemoBase.cpp
	  ../Common/DemoBase.h

	  ${VIS_FILES}
	  ${PROJECT_PATH}/Common/Common.h

	  CMakeLists.txt
)

set_target_properties(SutureVisualDemo PROPERTIES FOLDER "Demos")
set_target_properties(SutureVisualDemo PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(SutureVisualDemo PROPERTIES RELWITHDEBINFO_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
set_target_properties(SutureVisualDemo PROPERTIES MINSIZEREL_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
PBD_CONFIGURE_DEMO_TARGET(SutureVisualDemo)

add_executable(SpineWorkflowDemo
	  SpineWorkflowDemo.cpp
	  WorkflowSceneAssets.cpp
	  WorkflowSceneAssets.h
	  WorkflowRuntimePolicy.h
	  WorkflowSdfGrinding.h
	  WorkflowSceneLayerStyle.h
	  WorkflowSceneTextureMapping.h
	  WorkflowSoftTissueFracture.h

	  ../Common/LogWindow.cpp
	  ../Common/LogWindow.h
	  ../Common/Simulator_GUI_imgui.cpp
	  ../Common/Simulator_GUI_imgui.h
	  ../Common/imguiParameters.cpp
	  ../Common/imguiParameters.h
	  ../Common/DemoBase.cpp
	  ../Common/DemoBase.h

	  ${VIS_FILES}
	  ${PROJECT_PATH}/Common/Common.h
	  ${PROJECT_PATH}/Demos/Visualization/MC.h

	  CMakeLists.txt
)

set_target_properties(SpineWorkflowDemo PROPERTIES FOLDER "Demos")
set_target_properties(SpineWorkflowDemo PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(SpineWorkflowDemo PROPERTIES RELWITHDEBINFO_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
set_target_properties(SpineWorkflowDemo PROPERTIES MINSIZEREL_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
PBD_CONFIGURE_DEMO_TARGET(SpineWorkflowDemo)

add_executable(TetModelInitializationBenchmark
	  selfTest/TetModelInitializationBenchmark.cpp

	  ${PROJECT_PATH}/Common/Common.h

	  CMakeLists.txt
)

set_target_properties(TetModelInitializationBenchmark PROPERTIES FOLDER "Demos")
set_target_properties(TetModelInitializationBenchmark PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(TetModelInitializationBenchmark PROPERTIES RELWITHDEBINFO_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
set_target_properties(TetModelInitializationBenchmark PROPERTIES MINSIZEREL_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
target_include_directories(TetModelInitializationBenchmark PRIVATE ${Discregrid_INCLUDE_DIR})
target_link_libraries(TetModelInitializationBenchmark Simulation Utils ${Discregrid_LIBRARIES})
if (TARGET Ext_Discregrid)
	add_dependencies(TetModelInitializationBenchmark Ext_Discregrid)
endif()

add_executable(TetModelFractureStabilityBenchmark
	  selfTest/TetModelFractureStabilityBenchmark.cpp

	  ${PROJECT_PATH}/Common/Common.h

	  CMakeLists.txt
)

set_target_properties(TetModelFractureStabilityBenchmark PROPERTIES FOLDER "Demos")
set_target_properties(TetModelFractureStabilityBenchmark PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(TetModelFractureStabilityBenchmark PROPERTIES RELWITHDEBINFO_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
set_target_properties(TetModelFractureStabilityBenchmark PROPERTIES MINSIZEREL_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
target_include_directories(TetModelFractureStabilityBenchmark PRIVATE ${Discregrid_INCLUDE_DIR})
target_link_libraries(TetModelFractureStabilityBenchmark Simulation Utils ${Discregrid_LIBRARIES})
if (TARGET Ext_Discregrid)
	add_dependencies(TetModelFractureStabilityBenchmark Ext_Discregrid)
endif()

if(BUILD_TESTING)
	add_test(NAME DemoFlowSummaryTest COMMAND $<TARGET_FILE:DemoFlowSummaryTest>)
	add_test(NAME DemoGuiStartupPolicyTest COMMAND $<TARGET_FILE:DemoGuiStartupPolicyTest>)
	add_test(NAME FindEdgeKeyTest COMMAND $<TARGET_FILE:FindEdgeKeyTest>)
	add_test(NAME LegacyToolPathGateTest COMMAND $<TARGET_FILE:LegacyToolPathGateTest>)
	add_test(NAME ToolDraggedFragmentSelectionTest COMMAND $<TARGET_FILE:ToolDraggedFragmentSelectionTest>)
	add_test(NAME RigidBodyMeshUpdateTest COMMAND $<TARGET_FILE:RigidBodyMeshUpdateTest>)
	add_test(NAME SDFSurfaceReconstructionHelperTest COMMAND $<TARGET_FILE:SDFSurfaceReconstructionHelperTest>)
	add_test(NAME TextureAssetDiagnosticsTest COMMAND $<TARGET_FILE:TextureAssetDiagnosticsTest>)
	add_test(NAME LiveHapticToolControlTest COMMAND $<TARGET_FILE:LiveHapticToolControlTest>)
	add_test(NAME WorkflowSceneTextureMappingTest COMMAND $<TARGET_FILE:WorkflowSceneTextureMappingTest>)
	add_test(NAME WorkflowRuntimePolicyTest COMMAND $<TARGET_FILE:WorkflowRuntimePolicyTest>)
	add_test(NAME WorkflowSceneLayerStyleTest COMMAND $<TARGET_FILE:WorkflowSceneLayerStyleTest>)
	add_test(NAME WorkflowKeyboardControlsTest COMMAND $<TARGET_FILE:WorkflowKeyboardControlsTest>)
	add_test(NAME ToolCutInteractionTest COMMAND $<TARGET_FILE:ToolCutInteractionTest>)
	add_test(NAME ToolCutDiagnosticsTest COMMAND $<TARGET_FILE:ToolCutDiagnosticsTest>)
	add_test(NAME TetModelPipelineBenchmark COMMAND $<TARGET_FILE:TetModelPipelineBenchmark>)
	add_test(NAME TetModelSutureBenchmark COMMAND $<TARGET_FILE:TetModelSutureBenchmark>)
	add_test(NAME SutureDemoSyntheticSmoke COMMAND $<TARGET_FILE:SutureDemo> --headless-smoke)
	add_test(NAME SutureDemoDetailedProcess COMMAND $<TARGET_FILE:SutureDemo> --detailed-process)
	add_test(NAME SpineWorkflowDemoFixedSummarySmoke
		COMMAND $<TARGET_FILE:SpineWorkflowDemo> --workflow-fixed-summary --demo-flow-summary)
	add_test(NAME SpineWorkflowDemoGuiStateSmoke
		COMMAND $<TARGET_FILE:SpineWorkflowDemo> --workflow-gui-state-smoke --demo-flow-summary)
	add_test(NAME SpineWorkflowDemoAutoDemoSmoke
		COMMAND $<TARGET_FILE:SpineWorkflowDemo> --workflow-auto-demo-smoke --demo-flow-summary)
	add_test(NAME SpineWorkflowDemoAutoDemoContractSmoke
		COMMAND $<TARGET_FILE:SpineWorkflowDemo> --workflow-auto-demo-smoke --demo-flow-summary)
	add_test(NAME SpineWorkflowDemoVisualSceneSmoke
		COMMAND $<TARGET_FILE:SpineWorkflowDemo> --workflow-auto-demo-smoke --demo-flow-summary)
	add_test(NAME SpineWorkflowDemoVisualFixSmoke
		COMMAND $<TARGET_FILE:SpineWorkflowDemo> --workflow-auto-demo-smoke --demo-flow-summary)
	add_test(NAME SpineWorkflowDemoSoftStageIsolationSmoke
		COMMAND $<TARGET_FILE:SpineWorkflowDemo> --workflow-auto-demo-smoke --demo-flow-summary)
	add_test(NAME SpineWorkflowDemoDiscReferenceTransformSmoke
		COMMAND $<TARGET_FILE:SpineWorkflowDemo> --workflow-auto-demo-smoke --demo-flow-summary)
	add_test(NAME SpineWorkflowDemoSjlgRetractionSmoke
		COMMAND $<TARGET_FILE:SpineWorkflowDemo> --workflow-auto-demo-smoke --demo-flow-summary)
	add_test(NAME SpineWorkflowDemoVisualPerformanceSmoke
		COMMAND $<TARGET_FILE:SpineWorkflowDemo> --workflow-minigl-screenshot-smoke --demo-flow-summary)
	add_test(NAME SpineWorkflowDemoLivePerformanceSmoke
		COMMAND $<TARGET_FILE:SpineWorkflowDemo> --workflow-live-performance-smoke --demo-flow-summary)
	add_test(NAME FractureDemoToolCutGuiStateSmoke
		COMMAND $<TARGET_FILE:FractureDemo> --tool-cut-gui-state-smoke --demo-flow-summary)
	add_test(NAME SpineDemoToolCutGuiStateSmoke
		COMMAND $<TARGET_FILE:SpineDemo> --tool-cut-gui-state-smoke --demo-flow-summary)
	add_test(NAME TetModelFractureStability-CrossFixture
		COMMAND $<TARGET_FILE:TetModelFractureStabilityBenchmark>
			--scenario fracture-target-disc
			--resolution 30 30 20
			--cross-attachment-fixture
			--min-fps 1
			--hard-gate-all)
	add_test(NAME TetModelFractureStability-RecordedDualComponentReplay
		COMMAND $<TARGET_FILE:TetModelFractureStabilityBenchmark>
			--scenario fracture-target-disc
			--resolution 30 30 20
			--hit-path-replay data/selftest/tetmodel/fracture-target-disc-recorded-dual-component-hit-path.json
			--require-dual-component-replay
			--min-fps 1
			--hard-gate-all)
	add_test(NAME TetModelFractureStability-MultiFracture
		COMMAND $<TARGET_FILE:TetModelFractureStabilityBenchmark>
			--scenario fracture-target-disc
			--resolution 30 30 20
			--multi-fracture-count 3
			--min-fps 1
			--hard-gate-all)
	set_tests_properties(
		DemoFlowSummaryTest
		DemoGuiStartupPolicyTest
		FindEdgeKeyTest
		LegacyToolPathGateTest
		ToolDraggedFragmentSelectionTest
		RigidBodyMeshUpdateTest
		SDFSurfaceReconstructionHelperTest
		TextureAssetDiagnosticsTest
		ToolCutInteractionTest
		ToolCutDiagnosticsTest
		TetModelPipelineBenchmark
		TetModelSutureBenchmark
		SutureDemoSyntheticSmoke
		SutureDemoDetailedProcess
		SpineWorkflowDemoFixedSummarySmoke
		SpineWorkflowDemoGuiStateSmoke
		SpineWorkflowDemoAutoDemoSmoke
		SpineWorkflowDemoAutoDemoContractSmoke
		SpineWorkflowDemoVisualSceneSmoke
		SpineWorkflowDemoVisualFixSmoke
		SpineWorkflowDemoSoftStageIsolationSmoke
		SpineWorkflowDemoDiscReferenceTransformSmoke
		SpineWorkflowDemoSjlgRetractionSmoke
		SpineWorkflowDemoVisualPerformanceSmoke
		SpineWorkflowDemoLivePerformanceSmoke
		FractureDemoToolCutGuiStateSmoke
		SpineDemoToolCutGuiStateSmoke
		TetModelFractureStability-CrossFixture
		TetModelFractureStability-RecordedDualComponentReplay
		TetModelFractureStability-MultiFracture
		PROPERTIES WORKING_DIRECTORY ${PROJECT_PATH})
	set_tests_properties(
		SutureDemoSyntheticSmoke
		PROPERTIES PASS_REGULAR_EXPRESSION "suture_demo_status=ok")
	set_tests_properties(
		SutureDemoDetailedProcess
		PROPERTIES PASS_REGULAR_EXPRESSION "suture_detailed_status=ok")
	set_tests_properties(
		SpineWorkflowDemoFixedSummarySmoke
		PROPERTIES
			PASS_REGULAR_EXPRESSION "workflow_fixed_summary_status=ok"
			TIMEOUT 5)
	set_tests_properties(
		SpineWorkflowDemoGuiStateSmoke
		PROPERTIES
			PASS_REGULAR_EXPRESSION "workflow_gui_state_smoke_status=ok"
			FAIL_REGULAR_EXPRESSION "workflow_gui_state_controls_registered=1([^0-9]|$)"
			TIMEOUT 5)
	set_tests_properties(
			SpineWorkflowDemoAutoDemoSmoke
			PROPERTIES
				PASS_REGULAR_EXPRESSION "workflow_auto_demo_status=ok"
				FAIL_REGULAR_EXPRESSION "workflow_auto_demo_ligament_inactive_constraints=0([^0-9]|$)|workflow_auto_demo_disc_inactive_constraints=0([^0-9]|$)|workflow_auto_demo_ligament_render_faces=0([^0-9]|$)|workflow_auto_demo_disc_render_faces=0([^0-9]|$)|workflow_auto_demo_ligament_detached_fragment_moved=0([^0-9]|$)|workflow_auto_demo_disc_detached_fragment_moved=0([^0-9]|$)|workflow_auto_demo_ligament_components=0([^0-9]|$)|workflow_auto_demo_ligament_components=1([^0-9]|$)|workflow_auto_demo_disc_components=0([^0-9]|$)|workflow_auto_demo_disc_components=1([^0-9]|$)|workflow_auto_demo_legacy_path_used=1([^0-9]|$)"
				TIMEOUT 10)
	set_tests_properties(
		SpineWorkflowDemoAutoDemoContractSmoke
		PROPERTIES
			PASS_REGULAR_EXPRESSION "workflow_auto_demo_contract_status=ok"
			FAIL_REGULAR_EXPRESSION "workflow_auto_demo_ligament_result_moved=1([^0-9]|$)|workflow_auto_demo_sjlg_hidden=0([^0-9]|$)|workflow_auto_demo_retractor_asset_is_sjlg=1([^0-9]|$)|workflow_auto_demo_retractor_moved=1([^0-9]|$)|workflow_auto_demo_gui_controls_registered=1([^0-9]|$)|workflow_auto_demo_preserved_replaces_default_anatomy=0([^0-9]|$)"
			TIMEOUT 10)
	set_tests_properties(
		SpineWorkflowDemoVisualSceneSmoke
		PROPERTIES
			PASS_REGULAR_EXPRESSION "workflow_auto_demo_visual_scene_status=ok"
			FAIL_REGULAR_EXPRESSION "workflow_auto_demo_scene_layers_loaded=0([^0-9]|$)|workflow_auto_demo_visual_scene_replaces_default_targets=0([^0-9]|$)"
			TIMEOUT 10)
		set_tests_properties(
			SpineWorkflowDemoVisualFixSmoke
			PROPERTIES
				PASS_REGULAR_EXPRESSION "workflow_auto_demo_visible_artifact_status=ok"
				FAIL_REGULAR_EXPRESSION "workflow_auto_demo_scene_layers_loaded=0([^0-9]|$)|workflow_auto_demo_back_layer_loaded=0([^0-9]|$)|workflow_auto_demo_muscle_layer_loaded=0([^0-9]|$)|workflow_auto_demo_bone_grind_top_contact=0([^0-9]|$)|workflow_auto_demo_ligament_grasp_phase_seen=0([^0-9]|$)|workflow_auto_demo_ligament_pull_phase_seen=0([^0-9]|$)|workflow_auto_demo_disc_grasp_phase_seen=0([^0-9]|$)|workflow_auto_demo_disc_pull_phase_seen=0([^0-9]|$)|workflow_auto_demo_soft_visual_fracture_status=failed|workflow_auto_demo_sjlg_retraction_status=failed|workflow_auto_demo_sjlg_hidden=0([^0-9]|$)|workflow_auto_demo_retractor_moved=1([^0-9]|$)|workflow_auto_demo_golden_reference_status=failed|workflow_auto_demo_stage1_ball_grinder_proxy_status=failed|workflow_auto_demo_stage1_visible_grind_status=failed|workflow_auto_demo_bone_grind_patch_status=failed|workflow_auto_demo_stage2_golden_script_status=failed|workflow_auto_demo_stage2_hold_status=failed|workflow_auto_demo_stage2_main_component_capture_status=failed|workflow_auto_demo_stage2_ligament_visual_scale_status=failed|workflow_auto_demo_stage3_golden_script_status=failed|workflow_auto_demo_stage3_live_hold_status=failed|workflow_auto_demo_stage3_fragment_patch_status=failed|workflow_auto_demo_context_visibility_status=failed|workflow_auto_demo_visible_artifact_status=failed"
				TIMEOUT 10)
	set_tests_properties(
		SpineWorkflowDemoSoftStageIsolationSmoke
		PROPERTIES
			PASS_REGULAR_EXPRESSION "workflow_auto_demo_soft_stage_isolation_status=ok"
			FAIL_REGULAR_EXPRESSION "workflow_auto_demo_stage2_tet_model_count=0([^0-9]|$)|workflow_auto_demo_stage2_tet_model_count=2([^0-9]|$)|workflow_auto_demo_stage3_tet_model_count=0([^0-9]|$)|workflow_auto_demo_stage3_tet_model_count=2([^0-9]|$)|workflow_auto_demo_stage2_ligament_interactive=0([^0-9]|$)|workflow_auto_demo_stage2_disc_interactive=1([^0-9]|$)|workflow_auto_demo_stage2_disc_render_only=0([^0-9]|$)|workflow_auto_demo_stage3_ligament_interactive=1([^0-9]|$)|workflow_auto_demo_stage3_disc_interactive=0([^0-9]|$)|workflow_auto_demo_stage3_ligament_render_only=0([^0-9]|$)"
			TIMEOUT 10)
	set_tests_properties(
		SpineWorkflowDemoDiscReferenceTransformSmoke
		PROPERTIES
			PASS_REGULAR_EXPRESSION "workflow_auto_demo_disc_reference_transform_status=ok"
			FAIL_REGULAR_EXPRESSION "workflow_auto_demo_retractor_uses_disc_demo_scale=0([^0-9]|$)|workflow_auto_demo_stage3_tool_uses_disc_demo_scale=0([^0-9]|$)"
			TIMEOUT 10)
	set_tests_properties(
		SpineWorkflowDemoSjlgRetractionSmoke
		PROPERTIES
			PASS_REGULAR_EXPRESSION "workflow_auto_demo_sjlg_retraction_status=ok"
			FAIL_REGULAR_EXPRESSION "workflow_auto_demo_sjlg_hidden=0([^0-9]|$)|workflow_auto_demo_retractor_asset_is_sjlg=1([^0-9]|$)|workflow_auto_demo_retractor_moved=1([^0-9]|$)|workflow_auto_demo_ligament_result_moved=1([^0-9]|$)"
			TIMEOUT 10)
	set_tests_properties(
		SpineWorkflowDemoVisualPerformanceSmoke
		PROPERTIES
			PASS_REGULAR_EXPRESSION "workflow_minigl_screenshot_render_budget_status=ok"
			FAIL_REGULAR_EXPRESSION "workflow_minigl_screenshot_written=0([^0-9]|$)|workflow_minigl_screenshot_non_background_pixels=0([^0-9]|$)|workflow_minigl_screenshot_render_budget_status=failed"
			TIMEOUT 10)
	set_tests_properties(
		SpineWorkflowDemoLivePerformanceSmoke
		PROPERTIES
			PASS_REGULAR_EXPRESSION "workflow_live_performance_status=ok"
			FAIL_REGULAR_EXPRESSION "workflow_live_performance_reached_stage3=0([^0-9]|$)|workflow_live_performance_steps_per_render_status=failed|workflow_live_performance_idle_physics_paused=0([^0-9]|$)|workflow_live_performance_status=failed"
			TIMEOUT 10)
	set_tests_properties(
		FractureDemoToolCutGuiStateSmoke
		PROPERTIES
			PASS_REGULAR_EXPRESSION "tool_cut_gui_state_smoke_status=ok"
			TIMEOUT 5)
	set_tests_properties(
		SpineDemoToolCutGuiStateSmoke
		PROPERTIES
			PASS_REGULAR_EXPRESSION "tool_cut_gui_state_smoke_status=ok"
			TIMEOUT 5)
endif()

find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )
include_directories(${PROJECT_PATH}/extern/freeglut/include)
include_directories(${PROJECT_PATH}/extern/glew/include)


VIS_SOURCE_GROUPS()
