#include "WorkflowSceneAssets.h"

#include "Utils/FileSystem.h"

#include <array>

using namespace PBD;

namespace
{
	bool resourceExists(const std::string& exePath, const std::string& relativePath)
	{
		return Utilities::FileSystem::fileExists(
			Utilities::FileSystem::normalizePath(exePath + relativePath));
	}
}

WorkflowSceneAssets::ToolResolution WorkflowSceneAssets::resolveToolRelativePath(
	const std::string& exePath,
	const std::string& preferredToolAssetPath,
	const std::string& fallbackToolAssetPath,
	const bool allowFallback)
{
	ToolResolution result;
	const std::array<std::string, 2> preferredCandidates = {
		"/resources/models/" + preferredToolAssetPath,
		"/resources/models/scene/" + preferredToolAssetPath
	};
	for (const std::string& candidate : preferredCandidates)
	{
		if (resourceExists(exePath, candidate))
		{
			result.relativePath = candidate;
			result.resolvedAssetPath = preferredToolAssetPath;
			result.found = true;
			return result;
		}
	}

	if (!allowFallback || fallbackToolAssetPath.empty())
	{
		result.relativePath = preferredCandidates.back();
		result.resolvedAssetPath = preferredToolAssetPath;
		result.warning = "Required tool '" + preferredToolAssetPath + "' not found";
		return result;
	}

	const std::array<std::string, 2> fallbackCandidates = {
		"/resources/models/scene/" + fallbackToolAssetPath,
		"/resources/models/" + fallbackToolAssetPath
	};
	for (const std::string& candidate : fallbackCandidates)
	{
		if (resourceExists(exePath, candidate))
		{
			result.relativePath = candidate;
			result.resolvedAssetPath = fallbackToolAssetPath;
			result.warning = "Preferred tool '" + preferredToolAssetPath + "' not found; using '" + fallbackToolAssetPath + "'";
			result.found = true;
			result.usedFallback = true;
			return result;
		}
	}

	result.relativePath = fallbackCandidates.front();
	result.resolvedAssetPath = fallbackToolAssetPath;
	result.warning = "Preferred tool '" + preferredToolAssetPath + "' not found; fallback '" + fallbackToolAssetPath + "' is also missing";
	result.usedFallback = true;
	return result;
}

void WorkflowSceneAssets::clearAndLoadStageTool(
	DemoBase& demoBase,
	const std::string& toolRelativePath,
	const Vector3r& toolTranslation,
	const Vector3r& toolScale)
{
	for (unsigned int i = 0; i < demoBase.m_MaxSurgToolNbr; i++)
	{
		demoBase.m_surgToolMeshPairs[i].surgToolVDs[DemoBase::SurgToolMeshPair::STS_INACTIVE].release();
		demoBase.m_surgToolMeshPairs[i].surgToolMeshs[DemoBase::SurgToolMeshPair::STS_INACTIVE].release();
		demoBase.m_surgToolMeshPairs[i].surgToolVDs[DemoBase::SurgToolMeshPair::STS_ACTIVE].release();
		demoBase.m_surgToolMeshPairs[i].surgToolMeshs[DemoBase::SurgToolMeshPair::STS_ACTIVE].release();
	}
	demoBase.m_totalSurgToolNbr = 0;
	demoBase.loadSurgToolMeshPair(
		toolRelativePath,
		"",
		toolTranslation,
		Matrix3r::Identity(),
		toolScale);
}

bool WorkflowSceneAssets::loadRenderOnlySceneMesh(
	const std::string& exePath,
	const std::string& sceneAssetName,
	VertexData& vertices,
	Utilities::IndexedFaceMesh& mesh)
{
	const std::array<std::string, 2> candidates = {
		Utilities::FileSystem::normalizePath(exePath + "/resources/models/scene/" + sceneAssetName),
		Utilities::FileSystem::normalizePath(exePath + "/../../models/scene/" + sceneAssetName)
	};
	std::string fullPath;
	for (const std::string& candidate : candidates)
	{
		if (Utilities::FileSystem::fileExists(candidate))
		{
			fullPath = candidate;
			break;
		}
	}
	if (fullPath.empty())
		return false;
	DemoBase::loadMesh(fullPath, vertices, mesh, Vector3r::Zero(), Matrix3r::Identity(), Vector3r::Ones());
	return (vertices.size() > 0u) && (mesh.numFaces() > 0u);
}

bool WorkflowSceneAssets::isLgRetractorAsset(const std::string& assetName)
{
	return assetName == "lg.obj";
}

bool WorkflowSceneAssets::isSjlgRetractorAsset(const std::string& assetName)
{
	return assetName == "sjlg.obj";
}
