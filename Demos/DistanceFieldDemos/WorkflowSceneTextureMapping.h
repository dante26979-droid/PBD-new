#ifndef __WorkflowSceneTextureMapping_h__
#define __WorkflowSceneTextureMapping_h__

enum class WorkflowSceneTextureRole
{
	BoneReference = 0,
	NerveContext,
	OtherDiscContext,
	TargetDiscReference,
	MuscleContext,
	BackContext,
	LigamentReference
};

struct WorkflowSceneTextureSlot
{
	WorkflowSceneTextureSlot() :
		mapped(false),
		textureIndex(0u)
	{
	}

	WorkflowSceneTextureSlot(const bool mappedValue, const unsigned int textureIndexValue) :
		mapped(mappedValue),
		textureIndex(textureIndexValue)
	{
	}

	bool mapped;
	unsigned int textureIndex;
};

inline WorkflowSceneTextureSlot workflowSceneTextureSlot(const WorkflowSceneTextureRole role)
{
	switch (role)
	{
	case WorkflowSceneTextureRole::BoneReference:
		return { true, 1u };
	case WorkflowSceneTextureRole::NerveContext:
		return { true, 2u };
	case WorkflowSceneTextureRole::OtherDiscContext:
	case WorkflowSceneTextureRole::TargetDiscReference:
		return { true, 3u };
	case WorkflowSceneTextureRole::MuscleContext:
		return { true, 4u };
	case WorkflowSceneTextureRole::BackContext:
		return { true, 0u };
	case WorkflowSceneTextureRole::LigamentReference:
		return { true, 5u };
	default:
		return { false, 0u };
	}
}

#endif
