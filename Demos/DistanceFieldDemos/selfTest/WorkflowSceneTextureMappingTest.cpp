#include "Demos/DistanceFieldDemos/WorkflowSceneTextureMapping.h"

#include <iostream>

namespace
{
	bool expectTexture(
		const WorkflowSceneTextureRole role,
		const bool expectedMapped,
		const unsigned int expectedSlot,
		const char* label)
	{
		const WorkflowSceneTextureSlot slot = workflowSceneTextureSlot(role);
		if (slot.mapped != expectedMapped)
		{
			std::cerr << label << " mapped mismatch\n";
			return false;
		}
		if (slot.mapped && (slot.textureIndex != expectedSlot))
		{
			std::cerr << label << " texture index mismatch: expected "
				<< expectedSlot << ", got " << slot.textureIndex << "\n";
			return false;
		}
		return true;
	}
}

int main()
{
	bool ok = true;
	ok &= expectTexture(WorkflowSceneTextureRole::BoneReference, true, 1u, "bone");
	ok &= expectTexture(WorkflowSceneTextureRole::NerveContext, true, 2u, "nerve");
	ok &= expectTexture(WorkflowSceneTextureRole::OtherDiscContext, true, 3u, "other_disc");
	ok &= expectTexture(WorkflowSceneTextureRole::TargetDiscReference, true, 3u, "target_disc");
	ok &= expectTexture(WorkflowSceneTextureRole::MuscleContext, true, 4u, "muscle");
	ok &= expectTexture(WorkflowSceneTextureRole::BackContext, true, 0u, "back");
	ok &= expectTexture(WorkflowSceneTextureRole::LigamentReference, true, 5u, "ligament");

	if (!ok)
		return 1;

	std::cout << "WorkflowSceneTextureMappingTest ok\n";
	return 0;
}
