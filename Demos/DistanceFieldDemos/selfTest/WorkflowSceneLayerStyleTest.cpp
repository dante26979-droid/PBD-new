#include "Demos/DistanceFieldDemos/WorkflowSceneLayerStyle.h"

#include <cstdlib>
#include <iostream>

namespace
{
void require(bool condition, const char* message)
{
	if (!condition)
	{
		std::cerr << message << "\n";
		std::exit(1);
	}
}
}

int main()
{
	float color[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
	PBD::workflowSceneLayerColor(PBD::WorkflowSceneLayerRole::BackContext, color);
	require(color[3] == 1.0f, "skin/back context must render opaque.");

	PBD::workflowSceneLayerColor(PBD::WorkflowSceneLayerRole::MuscleContext, color);
	require(color[3] == 1.0f, "muscle context must render opaque.");

	PBD::workflowSceneLayerColor(PBD::WorkflowSceneLayerRole::BoneReference, color);
	require(color[0] == 0.5f && color[1] == 0.5f && color[2] == 0.5f && color[3] == 1.0f,
		"bone reference must use the independent grinding demo static material color.");

	PBD::workflowSceneLayerColor(PBD::WorkflowSceneLayerRole::LgRetractorContext, color);
	require(color[0] == 0.5f && color[1] == 0.5f && color[2] == 0.5f && color[3] == 1.0f,
		"lg retractor must use the same non-textured static display style as the independent demo scene bodies.");
	PBD::workflowSceneLayerColor(PBD::WorkflowSceneLayerRole::GrindingLgToolContext, color);
	require(color[0] == 0.5f && color[1] == 0.5f && color[2] == 0.5f && color[3] == 1.0f,
		"stage 1 lg tool context must use the independent grinding demo static material color.");

	unsigned int textureId = 99u;
	require(PBD::workflowSceneLayerTextureId(PBD::WorkflowSceneLayerRole::BackContext, textureId) && textureId == 0u,
		"skin/back context must bind the independent demo skin texture slot.");
	require(PBD::workflowSceneLayerTextureId(PBD::WorkflowSceneLayerRole::BoneReference, textureId) && textureId == 1u,
		"bone reference must bind the independent demo skeleton texture slot.");
	require(PBD::workflowSceneLayerTextureId(PBD::WorkflowSceneLayerRole::NerveContext, textureId) && textureId == 2u,
		"nerve context must bind the independent demo spinal-cord texture slot.");
	require(PBD::workflowSceneLayerTextureId(PBD::WorkflowSceneLayerRole::OtherDiscContext, textureId) && textureId == 3u,
		"other disc context must bind the independent demo disc texture slot.");
	require(PBD::workflowSceneLayerTextureId(PBD::WorkflowSceneLayerRole::TargetDiscReference, textureId) && textureId == 3u,
		"target disc reference must bind the independent demo disc texture slot.");
	require(PBD::workflowSceneLayerTextureId(PBD::WorkflowSceneLayerRole::MuscleContext, textureId) && textureId == 4u,
		"muscle context must bind the independent demo muscle texture slot.");
	require(PBD::workflowSceneLayerTextureId(PBD::WorkflowSceneLayerRole::LigamentReference, textureId) && textureId == 5u,
		"ligament reference must bind the independent demo ligament texture slot.");
	require(!PBD::workflowSceneLayerTextureId(PBD::WorkflowSceneLayerRole::LgRetractorContext, textureId),
		"sjlg/retractor context must not claim one of the independent demo anatomy texture slots.");
	require(!PBD::workflowSceneLayerTextureId(PBD::WorkflowSceneLayerRole::GrindingLgToolContext, textureId),
		"stage 1 lg tool context must render as a non-textured independent demo scene body.");

	PBD::workflowIndependentSurfaceColor(color);
	require(color[0] == 0.1f && color[1] == 0.4f && color[2] == 0.7f && color[3] == 1.0f,
		"interactive workflow targets after stage 1 must use the independent demo surface color.");

	PBD::workflowInteractiveSurfaceColor(PBD::WorkflowSceneDisplayStage::BoneGrinding, color);
	require(color[0] == 0.5f && color[1] == 0.5f && color[2] == 0.5f && color[3] == 1.0f,
		"stage 1 interactive bone must use the independent grinding demo static material color.");

	PBD::workflowInteractiveSurfaceColor(PBD::WorkflowSceneDisplayStage::LigamentRemoval, color);
	require(color[0] == 0.1f && color[1] == 0.4f && color[2] == 0.7f && color[3] == 1.0f,
		"stage 2 interactive ligament must keep the independent demo surface color.");

	PBD::workflowInteractiveSurfaceColor(PBD::WorkflowSceneDisplayStage::DiscRemoval, color);
	require(color[0] == 0.1f && color[1] == 0.4f && color[2] == 0.7f && color[3] == 1.0f,
		"stage 3 interactive disc must keep the independent demo surface color.");

	PBD::workflowIndependentToolColor(color);
	require(color[0] == 1.0f && color[1] == 1.0f && color[2] == 1.0f && color[3] == 1.0f,
		"workflow tools must use the independent demo surgical tool color.");

	require(!PBD::workflowInteractiveMeshUsesTexture(true),
		"the current stage interactive mesh must not render with texture.");
	require(PBD::workflowInteractiveMeshUsesTexture(false),
		"the same mesh may render with texture after it becomes a preserved/reference result.");
	require(PBD::workflowFullContextRendersActiveStageMesh(),
		"FullContext must render the workflow active mesh so the interactive bone/soft tissue remains visible.");
	require(PBD::workflowFullContextRendersWorkflowToolVisual(),
		"FullContext must render the workflow stage tool so stage 1 shows the ball-grinder tool.");
	require(PBD::workflowSoftStageRendersFractureDisplayMesh(false, 5301u),
		"soft stages must show the active soft-tissue mesh immediately after stage switch.");
	require(PBD::workflowSoftStageRendersFractureDisplayMesh(true, 5301u),
		"soft stages must render the fracture display mesh after a cut is applied.");
	require(!PBD::workflowSoftStageRendersFractureDisplayMesh(true, 0u),
		"soft stages must not render an empty fracture display mesh.");
	require(PBD::workflowReleasedSoftTissueShowsMainBodyOnly(),
		"released soft-tissue grasps should hide the detached fragment and keep the main body.");
	require(PBD::workflowSceneContextLayerVisible(
		PBD::WorkflowSceneLayerRole::BoneReference,
		PBD::WorkflowSceneDisplayStage::BoneGrinding,
		false,
		false,
		false),
		"stage 1 must keep the display bone context visible while rendering the interactive bone target on top.");
	require(PBD::workflowSceneContextLayerVisible(
		PBD::WorkflowSceneLayerRole::BoneReference,
		PBD::WorkflowSceneDisplayStage::BoneGrinding,
		true,
		false,
		false),
		"stage 1 must keep the complete display bone visible even after an interactive bone result exists.");
	require(PBD::workflowSceneContextLayerVisible(
		PBD::WorkflowSceneLayerRole::GrindingLgToolContext,
		PBD::WorkflowSceneDisplayStage::BoneGrinding,
		false,
		false,
		false),
		"stage 1 must keep the independent grinding demo lg scene tool.");
	require(!PBD::workflowSceneContextLayerVisible(
		PBD::WorkflowSceneLayerRole::GrindingLgToolContext,
		PBD::WorkflowSceneDisplayStage::LigamentRemoval,
		false,
		false,
		false),
		"lg scene tool must only be visible in the grinding stage.");
	require(PBD::workflowSceneContextLayerVisible(
		PBD::WorkflowSceneLayerRole::BoneReference,
		PBD::WorkflowSceneDisplayStage::LigamentRemoval,
		false,
		false,
		false),
		"stage 2 must keep textured bone context visible when stage 1 has no preserved bone result.");
	require(PBD::workflowSceneContextLayerVisible(
		PBD::WorkflowSceneLayerRole::BoneReference,
		PBD::WorkflowSceneDisplayStage::DiscRemoval,
		false,
		false,
		false),
		"stage 3 must keep textured bone context visible when stage 1 has no preserved bone result.");
	require(!PBD::workflowSceneContextLayerVisible(
		PBD::WorkflowSceneLayerRole::BoneReference,
		PBD::WorkflowSceneDisplayStage::LigamentRemoval,
		true,
		false,
		false),
		"stage 2 must hide textured bone context only when a real preserved bone result exists.");
	require(PBD::workflowSceneContextLayerVisible(
		PBD::WorkflowSceneLayerRole::LigamentReference,
		PBD::WorkflowSceneDisplayStage::DiscRemoval,
		false,
		false,
		false),
		"stage 3 must keep textured ligament context visible when stage 2 has no preserved ligament result.");
	require(!PBD::workflowSceneContextLayerVisible(
		PBD::WorkflowSceneLayerRole::LigamentReference,
		PBD::WorkflowSceneDisplayStage::DiscRemoval,
		false,
		true,
		false),
		"stage 3 must hide textured ligament context only when a real preserved ligament result exists.");
	require(PBD::workflowStagePreservesResult(
		PBD::WorkflowSceneDisplayStage::BoneGrinding,
		false,
		true),
		"stage 1 must preserve the current bone state even when no grinding operation happened.");
	require(PBD::workflowStagePreservesResult(
		PBD::WorkflowSceneDisplayStage::BoneGrinding,
		true,
		true),
		"stage 1 must preserve a bone result after a real grinding operation.");
	require(PBD::workflowStagePreservesResult(
		PBD::WorkflowSceneDisplayStage::LigamentRemoval,
		false,
		true),
		"stage 2 must preserve the current ligament state even when no bite operation happened.");
	require(!PBD::workflowStagePreservesResult(
		PBD::WorkflowSceneDisplayStage::LigamentRemoval,
		false,
		false),
		"a stage must not preserve an empty current state.");
	require(PBD::workflowFullContextSceneLayerVisible(
		PBD::WorkflowSceneLayerRole::BoneReference,
		PBD::WorkflowSceneDisplayStage::BoneGrinding,
		true),
		"FullContext stage 1 must keep non-interactive display bone visible.");
	require(PBD::workflowFullContextSceneLayerVisible(
		PBD::WorkflowSceneLayerRole::LigamentReference,
		PBD::WorkflowSceneDisplayStage::BoneGrinding,
		true),
		"FullContext stage 1 must keep yellow ligament reference visible.");
	require(!PBD::workflowSceneContextLayerVisible(
		PBD::WorkflowSceneLayerRole::LgRetractorContext,
		PBD::WorkflowSceneDisplayStage::RetractionTransition,
		true,
		true,
		false),
		"sjlg retractor must not be displayed during the stage 2 to stage 3 transition.");
	require(!PBD::workflowFullContextSceneLayerVisible(
		PBD::WorkflowSceneLayerRole::LgRetractorContext,
		PBD::WorkflowSceneDisplayStage::RetractionTransition,
		true),
		"FullContext must not display the sjlg retractor during the transition.");
	require(!PBD::workflowFullContextSceneLayerVisible(
		PBD::WorkflowSceneLayerRole::LigamentReference,
		PBD::WorkflowSceneDisplayStage::LigamentRemoval,
		true),
		"FullContext stage 2 must hide the ligament reference because the ligament is interactive.");
	require(!PBD::workflowFullContextSceneLayerVisible(
		PBD::WorkflowSceneLayerRole::TargetDiscReference,
		PBD::WorkflowSceneDisplayStage::DiscRemoval,
		true),
		"FullContext stage 3 must hide the target disc reference because the disc is interactive.");

	std::cout << "WorkflowSceneLayerStyleTest ok\n";
	return 0;
}
