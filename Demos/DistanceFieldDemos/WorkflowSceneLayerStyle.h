#ifndef __WorkflowSceneLayerStyle_h__
#define __WorkflowSceneLayerStyle_h__

namespace PBD
{
	enum class WorkflowSceneLayerRole
	{
		BoneReference = 0,
		NerveContext,
		OtherDiscContext,
		LigamentReference,
		TargetDiscReference,
		BackContext,
		MuscleContext,
		LgRetractorContext,
		GrindingLgToolContext
	};

	enum class WorkflowSceneDisplayStage
	{
		BoneGrinding = 0,
		LigamentRemoval,
		RetractionTransition,
		DiscRemoval,
		Completed
	};

	inline void workflowIndependentStaticColor(float color[4])
	{
		color[0] = 0.5f; color[1] = 0.5f; color[2] = 0.5f; color[3] = 1.0f;
	}

	inline void workflowInteractiveBoneColor(float color[4])
	{
		workflowIndependentStaticColor(color);
	}

	inline void workflowIndependentSurfaceColor(float color[4])
	{
		color[0] = 0.1f; color[1] = 0.4f; color[2] = 0.7f; color[3] = 1.0f;
	}

	inline void workflowInteractiveSurfaceColor(const WorkflowSceneDisplayStage stage, float color[4])
	{
		if (stage == WorkflowSceneDisplayStage::BoneGrinding)
		{
			workflowInteractiveBoneColor(color);
			return;
		}
		workflowIndependentSurfaceColor(color);
	}

	inline void workflowIndependentToolColor(float color[4])
	{
		color[0] = 1.0f; color[1] = 1.0f; color[2] = 1.0f; color[3] = 1.0f;
	}

	inline void workflowIndependentTexturedColor(float color[4])
	{
		color[0] = 1.0f; color[1] = 1.0f; color[2] = 1.0f; color[3] = 1.0f;
	}

	inline void workflowSceneLayerColor(const WorkflowSceneLayerRole role, float color[4])
	{
		switch (role)
		{
		case WorkflowSceneLayerRole::BoneReference:
			workflowIndependentStaticColor(color);
			break;
		case WorkflowSceneLayerRole::NerveContext:
		case WorkflowSceneLayerRole::OtherDiscContext:
		case WorkflowSceneLayerRole::LigamentReference:
		case WorkflowSceneLayerRole::TargetDiscReference:
		case WorkflowSceneLayerRole::BackContext:
		case WorkflowSceneLayerRole::MuscleContext:
			workflowIndependentTexturedColor(color);
			break;
		case WorkflowSceneLayerRole::LgRetractorContext:
		case WorkflowSceneLayerRole::GrindingLgToolContext:
			workflowIndependentStaticColor(color);
			break;
		default:
			workflowIndependentStaticColor(color);
			break;
		}
	}

	inline bool workflowSceneLayerTextureId(const WorkflowSceneLayerRole role, unsigned int& textureId)
	{
		switch (role)
		{
		case WorkflowSceneLayerRole::BackContext:
			textureId = 0u;
			return true;
		case WorkflowSceneLayerRole::BoneReference:
			textureId = 1u;
			return true;
		case WorkflowSceneLayerRole::NerveContext:
			textureId = 2u;
			return true;
		case WorkflowSceneLayerRole::OtherDiscContext:
		case WorkflowSceneLayerRole::TargetDiscReference:
			textureId = 3u;
			return true;
		case WorkflowSceneLayerRole::MuscleContext:
			textureId = 4u;
			return true;
		case WorkflowSceneLayerRole::LigamentReference:
			textureId = 5u;
			return true;
		case WorkflowSceneLayerRole::GrindingLgToolContext:
		default:
			textureId = 0u;
			return false;
		}
	}

	inline bool workflowInteractiveMeshUsesTexture(const bool isCurrentStageInteractiveMesh)
	{
		return !isCurrentStageInteractiveMesh;
	}

	inline bool workflowFullContextRendersActiveStageMesh()
	{
		return true;
	}

	inline bool workflowFullContextRendersWorkflowToolVisual()
	{
		return true;
	}

	inline bool workflowSoftStageRendersFractureDisplayMesh(
		const bool softCutApplied,
		const unsigned int softDisplayFaceCount)
	{
		(void)softCutApplied;
		return softDisplayFaceCount > 0u;
	}

	inline bool workflowReleasedSoftTissueShowsMainBodyOnly()
	{
		return true;
	}

	inline bool workflowStagePreservesResult(
		const WorkflowSceneDisplayStage stage,
		const bool stageEdited,
		const bool resultMeshReady)
	{
		(void)stageEdited;
		if (!resultMeshReady)
			return false;
		return (stage == WorkflowSceneDisplayStage::BoneGrinding) ||
			(stage == WorkflowSceneDisplayStage::LigamentRemoval) ||
			(stage == WorkflowSceneDisplayStage::DiscRemoval);
	}

	inline bool workflowSceneContextLayerVisible(
		const WorkflowSceneLayerRole role,
		const WorkflowSceneDisplayStage stage,
		const bool boneResultAvailable,
		const bool ligamentResultAvailable,
		const bool discResultAvailable)
	{
		switch (role)
		{
		case WorkflowSceneLayerRole::BoneReference:
			if (stage == WorkflowSceneDisplayStage::BoneGrinding)
				return true;
			return !boneResultAvailable && (stage != WorkflowSceneDisplayStage::Completed);
		case WorkflowSceneLayerRole::NerveContext:
		case WorkflowSceneLayerRole::OtherDiscContext:
			return stage != WorkflowSceneDisplayStage::Completed || discResultAvailable;
		case WorkflowSceneLayerRole::LigamentReference:
			return !ligamentResultAvailable &&
				(stage != WorkflowSceneDisplayStage::LigamentRemoval) &&
				(stage != WorkflowSceneDisplayStage::Completed);
		case WorkflowSceneLayerRole::TargetDiscReference:
			return !discResultAvailable &&
				(stage != WorkflowSceneDisplayStage::DiscRemoval) &&
				(stage != WorkflowSceneDisplayStage::Completed);
		case WorkflowSceneLayerRole::BackContext:
		case WorkflowSceneLayerRole::MuscleContext:
			return true;
		case WorkflowSceneLayerRole::GrindingLgToolContext:
			return stage == WorkflowSceneDisplayStage::BoneGrinding;
		case WorkflowSceneLayerRole::LgRetractorContext:
			return false;
		default:
			return false;
		}
	}

	inline bool workflowFullContextSceneLayerVisible(
		const WorkflowSceneLayerRole role,
		const WorkflowSceneDisplayStage stage,
		const bool defaultVisible)
	{
		if (!defaultVisible)
			return false;
		if (role == WorkflowSceneLayerRole::LgRetractorContext)
			return false;
		if (role == WorkflowSceneLayerRole::GrindingLgToolContext)
			return stage == WorkflowSceneDisplayStage::BoneGrinding;
		if ((stage == WorkflowSceneDisplayStage::LigamentRemoval) &&
			(role == WorkflowSceneLayerRole::LigamentReference))
		{
			return false;
		}
		if ((stage == WorkflowSceneDisplayStage::DiscRemoval) &&
			(role == WorkflowSceneLayerRole::TargetDiscReference))
		{
			return false;
		}
		return true;
	}
}

#endif
