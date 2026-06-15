#ifndef WORKFLOW_SCENE_ASSETS_H
#define WORKFLOW_SCENE_ASSETS_H

#include "Common/Common.h"
#include "Demos/Common/DemoBase.h"

#include <string>

namespace PBD
{
namespace WorkflowSceneAssets
{
	struct ToolResolution
	{
		std::string relativePath;
		std::string resolvedAssetPath;
		std::string warning;
		bool found = false;
		bool usedFallback = false;
	};

	ToolResolution resolveToolRelativePath(
		const std::string& exePath,
		const std::string& preferredToolAssetPath,
		const std::string& fallbackToolAssetPath,
		const bool allowFallback);

	void clearAndLoadStageTool(
		DemoBase& demoBase,
		const std::string& toolRelativePath,
		const Vector3r& toolTranslation,
		const Vector3r& toolScale);

	bool loadRenderOnlySceneMesh(
		const std::string& exePath,
		const std::string& sceneAssetName,
		VertexData& vertices,
		Utilities::IndexedFaceMesh& mesh);

	bool isLgRetractorAsset(const std::string& assetName);
	bool isSjlgRetractorAsset(const std::string& assetName);
}
}

#endif
