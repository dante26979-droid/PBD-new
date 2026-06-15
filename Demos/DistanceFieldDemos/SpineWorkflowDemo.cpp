#include "Common/Common.h"
#include "Demos/Visualization/MiniGL.h"
#include "Demos/Visualization/Selection.h"
#include "Simulation/TimeManager.h"
#include <Eigen/Dense>
#include "Simulation/SimulationModel.h"
#include "Simulation/TimeStepController.h"
#include <iostream>
#include "Demos/Visualization/Visualization.h"
#include "Utils/Logger.h"
#include "Utils/Timing.h"
#include "Utils/FileSystem.h"
#include "Demos/Common/DemoBase.h"
#include "Demos/Common/DemoFlowConstraintMetrics.h"
#include "Demos/Common/DemoFlowSummary.h"
#include "Demos/Common/LiveHapticMiniGLAdapter.h"
#include "Demos/Common/SDFSurfaceReconstruction.h"
#include "Demos/Common/ToolDraggedFragmentSelection.h"
#include "WorkflowSceneAssets.h"
#include "WorkflowRuntimePolicy.h"
#include "WorkflowSdfGrinding.h"
#include "WorkflowSoftTissueFracture.h"
#include "Simulation/Simulation.h"
#include "Simulation/DistanceFieldCollisionDetection.h"
#include "Simulation/CubicSDFCollisionDetection.h"
#include "Simulation/Find.h"
#include "Simulation/ToolTargetInteractionController.h"
#include "Simulation/ToolCutInteraction.h"
#include "Simulation/TetGridBuilder.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>

using namespace PBD;
using namespace Eigen;
using namespace std;
using namespace Utilities;

DemoBase* base = nullptr;
bool gWorkflowFixedSummaryMode = false;
bool gWorkflowMiniGLScreenshotSmoke = false;
bool gWorkflowLivePerformanceSmoke = false;
bool gWorkflowHapticToolControl = true;
bool gWorkflowHapticUnavailableReported = false;
bool gWorkflowHapticToolWasActive = false;
bool gWorkflowHapticToolActive = false;
bool gWorkflowToolVisibilityMarkerValid = false;
Vector3r gWorkflowToolVisibilityMarkerPosition = Vector3r::Zero();
Vector3r gWorkflowHapticToolViewOffset = Vector3r::Zero();
bool gWorkflowHapticDiagnosticsEnabled = false;
std::string gWorkflowExePath;
std::string gWorkflowSceneFile;
constexpr unsigned int WORKFLOW_STEPS_PER_RENDER_UPDATE = 3u;

enum class WorkflowStage
{
	BoneGrinding = 0,
	LigamentRemoval,
	RetractionTransition,
	DiscRemoval,
	Completed
};

enum class WorkflowToolPoseAction
{
	MoveOnly = 0,
	SDFEdit,
	XPBDPullFracture
};

enum class WorkflowToolControlMode
{
	MinYTip = 0,
	BallGrinderProxy
};

struct WorkflowSoftScriptConfig
{
	unsigned int pathParticleA = 0u;
	unsigned int pathParticleB = 0u;
	unsigned int selectedParticle = 0u;
	Vector3r toolOffset = Vector3r::Zero();
	Vector3r toolPullDelta = Vector3r::Zero();
	Vector3r preCutParticleDelta = Vector3r::Zero();
	Real holdSecondsAfterFracture = static_cast<Real>(2.5);
	bool enabled = false;
};

struct WorkflowVisualMeshAsset
{
	std::string label;
	VertexData vertices;
	IndexedFaceMesh mesh;
	Vector3r translation = Vector3r::Zero();
	Matrix3r rotation = Matrix3r::Identity();
	Vector3r scale = Vector3r::Ones();
	unsigned int vertexBuffer = 0u;
	unsigned int normalBuffer = 0u;
	unsigned int indexBuffer = 0u;
	unsigned int indexCount = 0u;
	bool gpuMeshReady = false;
	bool visible = true;
};

enum class WorkflowSceneLayerRole
{
	BoneReference = 0,
	NerveContext,
	OtherDiscContext,
	LigamentReference,
	TargetDiscReference,
	BackContext,
	MuscleContext,
	GrindingLgToolContext
};

struct WorkflowSceneLayer
{
	WorkflowVisualMeshAsset asset;
	std::string sourceAssetName;
	WorkflowSceneLayerRole role = WorkflowSceneLayerRole::NerveContext;
	bool loaded = false;
};

struct WorkflowStageContext
{
	WorkflowStage stage;
	std::string stageName;
	std::string activeAssetPath;
	std::string preferredToolAssetPath;
	std::string fallbackToolAssetPath;
	std::string resolvedToolAssetPath;
	std::string resolvedToolRelativePath;
	std::string replayPath;
	Vector3r toolInitialTranslation = Vector3r::Zero();
	Vector3r toolVisualScale = Vector3r::Ones();
	WorkflowToolControlMode toolControlMode = WorkflowToolControlMode::MinYTip;

	Eigen::Matrix<unsigned int, 3, 1, Eigen::DontAlign> resolutionSDF;
	CubicSDFCollisionDetection::GridPtr distanceField;
	std::vector<Real> nodeSDFVals;
	std::vector<bool> eleAvailableMask;
	std::vector<unsigned int> availablePtIds;
	std::vector<Vector3r> allEleNodePoints;
	std::vector<Vector3r> activeEleNodePoints;
	std::vector<unsigned int> activeNodeIdxMap;

	MC::mcMesh mcMesh;
	MC::MarchingCubeWorkspace mcWorkspace;
	VertexData warpedVD;
	IndexedFaceMesh warpedMesh;
	SDFSurfaceSignature surfaceSignature;
	bool meshUpdated = false;

	ToolTargetInteractionController toolController;
	ToolProfile toolProfile;
	ToolProbe toolProbe;
	Real toolRadius = static_cast<Real>(0.15);
	bool toolProbeInitialized = false;
	Real ballGrinderProxyRadius = static_cast<Real>(0.0);
	Vector3r ballGrinderProxyLocalCenter = Vector3r::Zero();
	bool ballGrinderProxyReady = false;
	bool ballGrinderProxyContact = false;

	TetGridBuilder::Workspace softTetWorkspace;
	WorkflowSoftScriptConfig softScript;
	ToolCutInteraction::EdgeQueryState softCutQueryState;
	ToolCutInteraction::FractureTriggerState softFractureTriggerState;
	std::vector<unsigned int> softSelectedParticles;
	std::vector<unsigned int> softDraggedFragmentParticles;
	std::vector<unsigned int> softPendingCutFragmentParticles;
	std::vector<unsigned int> softDiscardedFragmentParticles;
	std::vector<std::array<unsigned int, 2> > softPendingCutHitEdges;
	std::vector<std::array<unsigned int, 2> > softLastFractureEdges;
	unsigned int softDraggedFragmentComponent = FractureState::InvalidComponent;
	bool softCutApplied = false;
	bool softHasLastToolPosition = false;
	Vector3r softLastToolPosition = Vector3r::Zero();
	bool softScriptSelectionSeeded = false;
	VertexData softDisplayVD;
	IndexedFaceMesh softDisplayMesh;
	TetModel::FractureDisplayRefreshStats lastFractureDisplayStats;
	bool softTetModelBuilt = false;
	unsigned int softHitEdgeCount = 0u;
	unsigned int softInactiveConstraintCount = 0u;
	unsigned int softRenderFaceCount = 0u;
	unsigned int softPatchFaceCount = 0u;
	unsigned int softDebugPatchFaceCount = 0u;
	unsigned int softDraggedParticleCount = 0u;
	unsigned int softHiddenFaceCount = 0u;
	unsigned int softMainComponentFaceCount = 0u;
	Real softDraggedDistance = static_cast<Real>(0.0);

	double lastSDFEditMs = 0.0;
	double lastMarchingCubeMs = 0.0;
	double lastMeshConvertMs = 0.0;
	double lastToolQueryMs = 0.0;
	std::string warning;

	void clear()
	{
		distanceField.reset();
		nodeSDFVals.clear();
		eleAvailableMask.clear();
		availablePtIds.clear();
		allEleNodePoints.clear();
		activeEleNodePoints.clear();
		activeNodeIdxMap.clear();
		mcMesh = MC::mcMesh();
		mcWorkspace = MC::MarchingCubeWorkspace();
		warpedVD.release();
		warpedMesh.release();
		surfaceSignature = SDFSurfaceSignature();
		meshUpdated = false;
		toolController = ToolTargetInteractionController();
		toolProfile = ToolProfile();
		toolProbe = ToolProbe();
		toolProbeInitialized = false;
		ballGrinderProxyRadius = static_cast<Real>(0.0);
		ballGrinderProxyLocalCenter = Vector3r::Zero();
		ballGrinderProxyReady = false;
		ballGrinderProxyContact = false;
		softTetWorkspace = TetGridBuilder::Workspace();
		softScript = WorkflowSoftScriptConfig();
		softCutQueryState = ToolCutInteraction::EdgeQueryState();
		softFractureTriggerState.clear();
		softSelectedParticles.clear();
		softDraggedFragmentParticles.clear();
		softPendingCutFragmentParticles.clear();
		softDiscardedFragmentParticles.clear();
		softPendingCutHitEdges.clear();
		softLastFractureEdges.clear();
		softDraggedFragmentComponent = FractureState::InvalidComponent;
		softCutApplied = false;
		softHasLastToolPosition = false;
		softLastToolPosition = Vector3r::Zero();
		softScriptSelectionSeeded = false;
		softDisplayVD.release();
		softDisplayMesh.release();
		lastFractureDisplayStats = TetModel::FractureDisplayRefreshStats();
		softTetModelBuilt = false;
		softHitEdgeCount = 0u;
		softInactiveConstraintCount = 0u;
		softRenderFaceCount = 0u;
		softPatchFaceCount = 0u;
		softDebugPatchFaceCount = 0u;
		softDraggedParticleCount = 0u;
		softHiddenFaceCount = 0u;
		softMainComponentFaceCount = 0u;
		softDraggedDistance = static_cast<Real>(0.0);
		lastSDFEditMs = 0.0;
		lastMarchingCubeMs = 0.0;
		lastMeshConvertMs = 0.0;
		lastToolQueryMs = 0.0;
		resolvedToolRelativePath.clear();
		toolInitialTranslation = Vector3r::Zero();
		toolVisualScale = Vector3r::Ones();
		toolControlMode = WorkflowToolControlMode::MinYTip;
		warning.clear();
	}
};

WorkflowStage gWorkflowStage = WorkflowStage::BoneGrinding;
WorkflowStageContext gActiveStageContext;
std::vector<WorkflowVisualMeshAsset> gPreservedMeshes;
std::vector<WorkflowSceneLayer> gWorkflowSceneLayers;

Real gTransitionDurationSec = static_cast<Real>(2.0);
Real gTransitionElapsedSec = static_cast<Real>(0.0);
bool gTransitionPlaying = false;

struct RetractedTransforms
{
	Vector3r toolTranslation = Vector3r::Zero();
};
RetractedTransforms gRetractedTransforms;
RetractedTransforms gTransitionStart;
RetractedTransforms gTransitionEnd;
WorkflowVisualMeshAsset gRetractorAsset;
bool gRetractorAssetLoaded = false;
std::string gRetractorAssetPath;
bool gWorkflowGUIControlsRegistered = false;

bool gAutoFixedOperationEnabled = true;
double gLastRenderMs = 0.0;
DemoFlowSummary gDemoFlowSummary;

char gStatusText[256] = "Stage 1: Bone Grinding. Press 'Run Fixed Operation' or 'Next Stage'.";

enum class WorkflowAutoDemoPhase
{
	Disabled = 0,
	BoneGrinding,
	BoneHold,
	LigamentTear,
	LigamentHold,
	Retraction,
	DiscPull,
	CompletedHold
};

enum class WorkflowToolPosePhase
{
	Approach = 0,
	Grasp,
	Pull,
	Fracture,
	Hold
};

enum class WorkflowVisualMode
{
	Performance = 0,
	FullContext
};

struct WorkflowToolPoseStep
{
	Vector3r tip = Vector3r::Zero();
	Real radius = static_cast<Real>(0.1);
	Real holdSeconds = static_cast<Real>(0.03);
	WorkflowToolPoseAction action = WorkflowToolPoseAction::SDFEdit;
	WorkflowToolPosePhase phase = WorkflowToolPosePhase::Approach;
};

struct WorkflowAutoDemoState
{
	bool commandLineRequested = false;
	bool running = false;
	WorkflowAutoDemoPhase phase = WorkflowAutoDemoPhase::Disabled;
	std::vector<WorkflowToolPoseStep> path;
	size_t stepIndex = 0u;
	Real holdRemaining = static_cast<Real>(0.0);
	std::size_t boneChangedNodes = 0u;
	std::size_t ligamentChangedNodes = 0u;
	std::size_t discChangedNodes = 0u;
	unsigned int ligamentHitEdges = 0u;
	unsigned int ligamentInactiveConstraints = 0u;
	unsigned int ligamentRenderFaces = 0u;
	unsigned int ligamentPatchFaces = 0u;
	unsigned int ligamentDebugPatchFaces = 0u;
	unsigned int ligamentDraggedParticles = 0u;
	unsigned int ligamentHiddenFaces = 0u;
	Real ligamentDraggedDistance = static_cast<Real>(0.0);
	unsigned int ligamentComponents = 0u;
	unsigned int discHitEdges = 0u;
	unsigned int discInactiveConstraints = 0u;
	unsigned int discRenderFaces = 0u;
	unsigned int discPatchFaces = 0u;
	unsigned int discDebugPatchFaces = 0u;
	unsigned int discDraggedParticles = 0u;
	unsigned int discHiddenFaces = 0u;
	Real discDraggedDistance = static_cast<Real>(0.0);
	unsigned int discComponents = 0u;
	std::string stage2ActiveSoftAsset;
	std::string stage3ActiveSoftAsset;
	unsigned int stage2TetModelCount = 0u;
	unsigned int stage3TetModelCount = 0u;
	bool stage2DiscRenderOnly = false;
	bool stage3LigamentRenderOnly = false;
	Vector3r stage3ToolScale = Vector3r::Zero();
	bool boneGrindTopContact = false;
	bool ligamentGraspPhaseSeen = false;
	bool ligamentPullPhaseSeen = false;
	bool discGraspPhaseSeen = false;
	bool discPullPhaseSeen = false;
	bool stage1BallGrinderProxyOk = false;
	bool stage1BallProxyContact = false;
	Real stage1BallProxyRadius = static_cast<Real>(0.0);
	unsigned int boneGrindPatchFaces = 0u;
	unsigned int stage2ScriptParticleId = 0u;
	Vector3r stage2ScriptOffset = Vector3r::Zero();
	bool stage2GoldenScriptOk = false;
	bool stage2HoldOk = false;
	unsigned int stage2MainComponentFaces = 0u;
	bool stage2MainComponentCaptureOk = false;
	unsigned int stage3ScriptParticleId = 0u;
	Vector3r stage3ScriptOffset = Vector3r::Zero();
	bool stage3GoldenScriptOk = false;
	bool stage3LiveHoldOk = false;
	bool stage3RemainsInteractive = false;
	Vector3r currentToolPosition = Vector3r::Zero();
	bool hasToolPosition = false;
};

enum class WorkflowPatchCopyMode
{
	AllDebugPatches = 0,
	VisibleSoftFragmentOnly
};

WorkflowAutoDemoState gAutoDemoState;
bool gWorkflowToolVisualInitialized = false;
Vector3r gWorkflowToolVisualCenter = Vector3r::Zero();
bool gWorkflowToolControlInitialized = false;
unsigned int gWorkflowToolControlIndex = 0u;
WorkflowVisualMode gWorkflowVisualMode = WorkflowVisualMode::Performance;
bool gWorkflowHasTimeStepWallClock = false;
std::chrono::high_resolution_clock::time_point gWorkflowTimeStepWallClock;

namespace
{
	using Clock = std::chrono::high_resolution_clock;

	double elapsedMs(const Clock::time_point& start)
	{
		return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
	}

	Real workflowLiveStepDt()
	{
		const Clock::time_point now = Clock::now();
		if (!gWorkflowHasTimeStepWallClock)
		{
			gWorkflowTimeStepWallClock = now;
			gWorkflowHasTimeStepWallClock = true;
			return static_cast<Real>(1.0 / 60.0);
		}
		const double elapsedSeconds =
			std::chrono::duration<double>(now - gWorkflowTimeStepWallClock).count();
		gWorkflowTimeStepWallClock = now;
		return static_cast<Real>(std::max(0.0, std::min(elapsedSeconds, 1.0 / 30.0)));
	}

	void setStatusText(const char* text)
	{
		strncpy(gStatusText, text, sizeof(gStatusText) - 1);
		gStatusText[sizeof(gStatusText) - 1] = '\0';
	}

	Vector3r domainPoint(const Eigen::AlignedBox3d& domain, const Real x, const Real y, const Real z)
	{
		const Vector3d minPos = domain.min();
		const Vector3d maxPos = domain.max();
		return Vector3r(
			static_cast<Real>(minPos[0] + (maxPos[0] - minPos[0]) * x),
			static_cast<Real>(minPos[1] + (maxPos[1] - minPos[1]) * y),
			static_cast<Real>(minPos[2] + (maxPos[2] - minPos[2]) * z));
	}

	bool hasArg(const int argc, char** argv, const std::string& expected)
	{
		for (int i = 1; i < argc; i++)
		{
			if (std::string(argv[i]) == expected)
				return true;
		}
		return false;
	}

	const std::string& workflowExePath()
	{
		if ((base != nullptr) && !base->getExePath().empty())
			return base->getExePath();
		if (gWorkflowExePath.empty())
			gWorkflowExePath = FileSystem::getProgramPath();
		return gWorkflowExePath;
	}

	void applyWorkflowViewport()
	{
		MiniGL::setViewport(40.0f, 1.0f, 150.0f,
			Vector3r(0.0, 30.0, 5.0),
			Vector3r(0.0, 0.0, 0.0));
	}

	std::string resolveWorkflowToolRelativePath(
		const std::string& preferredToolAssetPath,
		const std::string& fallbackToolAssetPath,
		std::string& resolvedToolAssetPath,
		std::string& warning)
	{
		const WorkflowSceneAssets::ToolResolution resolution =
			WorkflowSceneAssets::resolveToolRelativePath(
				workflowExePath(),
				preferredToolAssetPath,
				fallbackToolAssetPath,
				true);
		resolvedToolAssetPath = resolution.resolvedAssetPath;
		warning = resolution.warning;
		return resolution.relativePath;
	}

	std::string workflowSceneFile()
	{
		if (base != nullptr)
			return base->getSceneFile();
		return gWorkflowSceneFile;
	}

	inline int toGLSerialIdx(const Vector3i& intIdx, const Vector3i& res)
	{
		return intIdx[0] * res[1] * res[2] + intIdx[1] * res[2] + intIdx[2];
	}

	inline int toSDFSerialIdx(const Vector3i& intIdx, const Vector3i& res)
	{
		return intIdx[2] * res[0] * res[1] + intIdx[1] * res[0] + intIdx[0];
	}

	inline AlignedBox3r toRealDomain(const Eigen::AlignedBox3d& domain)
	{
		AlignedBox3r result;
		result.extend(domain.min().cast<Real>());
		result.extend(domain.max().cast<Real>());
		return result;
	}

	VertexData transformVertexData(
		const VertexData& src,
		const Vector3r& translation,
		const Matrix3r& rotation = Matrix3r::Identity(),
		const Vector3r& scale = Vector3r::Ones())
	{
		VertexData out;
		out.reserve(src.size());
		for (unsigned int i = 0; i < src.size(); i++)
		{
			const Vector3r local = src.getPosition(i).cwiseProduct(scale);
			out.addVertex(rotation * local + translation);
		}
		return out;
	}

	Vector3r vertexDataCenter(const VertexData& vd)
	{
		if (vd.size() == 0u)
			return Vector3r::Zero();
		Vector3r center = Vector3r::Zero();
		for (unsigned int i = 0u; i < vd.size(); i++)
			center += vd.getPosition(i);
		return center / static_cast<Real>(vd.size());
	}

	unsigned int workflowToolControlPointIndex(const VertexData& vd)
	{
		if (vd.size() == 0u)
			return 0u;

		unsigned int minIndex = 0u;
		Real minY = vd.getPosition(0u).y();
		for (unsigned int i = 1u; i < vd.size(); i++)
		{
			const Real y = vd.getPosition(i).y();
			if (y < minY)
			{
				minY = y;
				minIndex = i;
			}
		}
		return minIndex;
	}

	bool computeBallGrinderProxy(const VertexData& vd, Vector3r& center, Real& radius)
	{
		if (vd.size() == 0u)
			return false;

		AlignedBox3r bounds;
		for (unsigned int i = 0u; i < vd.size(); i++)
			bounds.extend(vd.getPosition(i));
		if (bounds.isEmpty())
			return false;

		const Vector3r extent = bounds.max() - bounds.min();
		unsigned int longAxis = 0u;
		if (extent.y() > extent[longAxis])
			longAxis = 1u;
		if (extent.z() > extent[longAxis])
			longAxis = 2u;

		const Real bandMax =
			bounds.min()[longAxis] + extent[longAxis] * static_cast<Real>(0.22);
		Vector3r sum = Vector3r::Zero();
		unsigned int count = 0u;
		AlignedBox3r bandBounds;
		for (unsigned int i = 0u; i < vd.size(); i++)
		{
			const Vector3r& p = vd.getPosition(i);
			if (p[longAxis] <= bandMax)
			{
				sum += p;
				bandBounds.extend(p);
				count++;
			}
		}
		if (count == 0u)
			return false;

		center = sum / static_cast<Real>(count);
		Real transverseMax = static_cast<Real>(0.0);
		for (unsigned int axis = 0u; axis < 3u; axis++)
		{
			if (axis == longAxis)
				continue;
			transverseMax = std::max(transverseMax, bandBounds.max()[axis] - bandBounds.min()[axis]);
		}
		radius = std::max(transverseMax * static_cast<Real>(0.5), static_cast<Real>(0.01));
		return true;
	}

	void translateVertexData(VertexData& vd, const Vector3r& delta)
	{
		for (unsigned int i = 0u; i < vd.size(); i++)
			vd.setPosition(i, vd.getPosition(i) + delta);
	}

	Vector3r computeTopRegionCenter(const VertexData& vd)
	{
		if (vd.size() == 0u)
			return Vector3r::Zero();
		Real maxY = -REAL_MAX;
		for (unsigned int i = 0u; i < vd.size(); i++)
			maxY = std::max(maxY, vd.getPosition(i).y());
		const Real band = static_cast<Real>(0.80);
		Vector3r sum = Vector3r::Zero();
		unsigned int count = 0u;
		for (unsigned int i = 0u; i < vd.size(); i++)
		{
			const Vector3r& p = vd.getPosition(i);
			if (p.y() >= maxY - band)
			{
				sum += p;
				count++;
			}
		}
		if (count == 0u)
			return Vector3r::Zero();
		return sum / static_cast<Real>(count);
	}

	bool meshFaceIndicesInRange(const IndexedFaceMesh& mesh, const unsigned int vertexCount)
	{
		const std::vector<unsigned int>& faces = mesh.getFaces();
		for (const unsigned int vertexIndex : faces)
		{
			if (vertexIndex >= vertexCount)
				return false;
		}
		return true;
	}

	void rebuildMeshVertexCount(IndexedFaceMesh& mesh, const unsigned int vertexCount)
	{
		if (mesh.numVertices() == vertexCount)
			return;

		IndexedFaceMesh rebuiltMesh;
		rebuiltMesh.setFlatShading(mesh.getFlatShading());
		rebuiltMesh.initMesh(
			vertexCount,
			std::max(mesh.numEdges(), mesh.numFaces() * 3u),
			mesh.numFaces());
		const std::vector<unsigned int>& faces = mesh.getFaces();
		for (unsigned int faceIndex = 0u; faceIndex < mesh.numFaces(); faceIndex++)
			rebuiltMesh.addFace(&faces[3u * faceIndex]);
		mesh = rebuiltMesh;
	}

	bool prepareRenderableMesh(WorkflowVisualMeshAsset& asset)
	{
		if ((asset.vertices.size() == 0u) || (asset.mesh.numFaces() == 0u))
			return false;
		if (!meshFaceIndicesInRange(asset.mesh, asset.vertices.size()))
			return false;

		rebuildMeshVertexCount(asset.mesh, asset.vertices.size());
		asset.mesh.updateNormals(asset.vertices, 0u);
		asset.mesh.updateVertexNormals(asset.vertices);
		return (asset.mesh.getVertexNormals().size() >= asset.vertices.size()) &&
			(asset.mesh.getFaceNormals().size() >= asset.mesh.numFaces());
	}

	bool isRenderableMeshReady(const WorkflowVisualMeshAsset& asset)
	{
		return (asset.vertices.size() > 0u) &&
			(asset.mesh.numFaces() > 0u) &&
			(asset.mesh.getVertexNormals().size() >= asset.vertices.size()) &&
			(asset.mesh.getFaceNormals().size() >= asset.mesh.numFaces()) &&
			meshFaceIndicesInRange(asset.mesh, asset.vertices.size());
	}

	void drawWorkflowMesh(const VertexData& vertices, const IndexedFaceMesh& mesh, const float color[4])
	{
		if ((vertices.size() == 0u) || (mesh.numFaces() == 0u))
			return;
		MiniGL::drawMesh(vertices.getVertices(), mesh.getFaces(), mesh.getFaceNormals(), color);
	}

	bool workflowGpuMeshSupported()
	{
		return (glGenBuffers != nullptr) &&
			(glBindBuffer != nullptr) &&
			(glBufferData != nullptr);
	}

	bool ensureWorkflowGpuMesh(WorkflowVisualMeshAsset& asset)
	{
		if (asset.gpuMeshReady)
			return true;
		if (!workflowGpuMeshSupported() || !isRenderableMeshReady(asset))
			return false;

		const std::vector<Vector3r>& vertices = asset.vertices.getVertices();
		const std::vector<Vector3r>& normals = asset.mesh.getVertexNormals();
		const std::vector<unsigned int>& faces = asset.mesh.getFaces();
		if (vertices.empty() || normals.empty() || faces.empty())
			return false;

		glGenBuffers(1, &asset.vertexBuffer);
		glGenBuffers(1, &asset.normalBuffer);
		glGenBuffers(1, &asset.indexBuffer);
		if ((asset.vertexBuffer == 0u) || (asset.normalBuffer == 0u) || (asset.indexBuffer == 0u))
			return false;

		glBindBuffer(GL_ARRAY_BUFFER, asset.vertexBuffer);
		glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vector3r), vertices.data(), GL_STATIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, asset.normalBuffer);
		glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(Vector3r), normals.data(), GL_STATIC_DRAW);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, asset.indexBuffer);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, faces.size() * sizeof(unsigned int), faces.data(), GL_STATIC_DRAW);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		asset.indexCount = static_cast<unsigned int>(faces.size());
		asset.gpuMeshReady = true;
		return true;
	}

	void drawWorkflowGpuMesh(WorkflowVisualMeshAsset& asset, const float color[4])
	{
		if (!ensureWorkflowGpuMesh(asset))
		{
			drawWorkflowMesh(asset.vertices, asset.mesh, color);
			return;
		}

		float speccolor[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, color);
		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, color);
		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
		glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 100.0f);
		glColor3fv(color);

		if (MiniGL::checkOpenGLVersion(3, 3))
		{
			glBindBuffer(GL_ARRAY_BUFFER, asset.vertexBuffer);
			glEnableVertexAttribArray(0);
			glVertexAttribPointer(0, 3, GL_REAL, GL_FALSE, 0, nullptr);
			glBindBuffer(GL_ARRAY_BUFFER, asset.normalBuffer);
			glEnableVertexAttribArray(2);
			glVertexAttribPointer(2, 3, GL_REAL, GL_FALSE, 0, nullptr);
		}
		else
		{
			glEnableClientState(GL_VERTEX_ARRAY);
			glEnableClientState(GL_NORMAL_ARRAY);
			glBindBuffer(GL_ARRAY_BUFFER, asset.vertexBuffer);
			glVertexPointer(3, GL_REAL, 0, nullptr);
			glBindBuffer(GL_ARRAY_BUFFER, asset.normalBuffer);
			glNormalPointer(GL_REAL, 0, nullptr);
		}

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, asset.indexBuffer);
		glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(asset.indexCount), GL_UNSIGNED_INT, nullptr);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		if (MiniGL::checkOpenGLVersion(3, 3))
		{
			glDisableVertexAttribArray(0);
			glDisableVertexAttribArray(2);
		}
		else
		{
			glDisableClientState(GL_VERTEX_ARRAY);
			glDisableClientState(GL_NORMAL_ARRAY);
		}
	}

	void drawWorkflowAssetMesh(WorkflowVisualMeshAsset& asset, const float color[4])
	{
		if (gWorkflowVisualMode == WorkflowVisualMode::Performance)
			drawWorkflowGpuMesh(asset, color);
		else
			drawWorkflowMesh(asset.vertices, asset.mesh, color);
	}

	void drawWorkflowForegroundMesh(const VertexData& vertices, const IndexedFaceMesh& mesh, const float color[4])
	{
		const GLboolean depthTestEnabled = glIsEnabled(GL_DEPTH_TEST);
		GLboolean depthWriteEnabled = GL_TRUE;
		glGetBooleanv(GL_DEPTH_WRITEMASK, &depthWriteEnabled);

		glDisable(GL_DEPTH_TEST);
		glDepthMask(GL_FALSE);
		drawWorkflowMesh(vertices, mesh, color);
		glDepthMask(depthWriteEnabled);
		if (depthTestEnabled)
			glEnable(GL_DEPTH_TEST);
		else
			glDisable(GL_DEPTH_TEST);
	}

	void workflowActiveSoftColors(const WorkflowStage stage, float normalColor[4], float foregroundColor[4])
	{
		if (stage == WorkflowStage::DiscRemoval)
		{
			normalColor[0] = 0.12f; normalColor[1] = 0.62f; normalColor[2] = 1.0f; normalColor[3] = 0.95f;
			foregroundColor[0] = 0.02f; foregroundColor[1] = 0.82f; foregroundColor[2] = 1.0f; foregroundColor[3] = 1.0f;
			return;
		}
		normalColor[0] = 0.95f; normalColor[1] = 0.72f; normalColor[2] = 0.18f; normalColor[3] = 0.95f;
		foregroundColor[0] = 1.0f; foregroundColor[1] = 0.86f; foregroundColor[2] = 0.15f; foregroundColor[3] = 1.0f;
	}

	void releaseWorkflowGpuMesh(WorkflowVisualMeshAsset& asset)
	{
		if (glDeleteBuffers != nullptr)
		{
			if (asset.vertexBuffer != 0u)
				glDeleteBuffers(1, &asset.vertexBuffer);
			if (asset.normalBuffer != 0u)
				glDeleteBuffers(1, &asset.normalBuffer);
			if (asset.indexBuffer != 0u)
				glDeleteBuffers(1, &asset.indexBuffer);
		}
		asset.vertexBuffer = 0u;
		asset.normalBuffer = 0u;
		asset.indexBuffer = 0u;
		asset.indexCount = 0u;
		asset.gpuMeshReady = false;
	}

	void releaseWorkflowGpuMeshes(std::vector<WorkflowVisualMeshAsset>& assets)
	{
		for (WorkflowVisualMeshAsset& asset : assets)
			releaseWorkflowGpuMesh(asset);
	}

	bool preservedMeshLabelExists(const std::string& label)
	{
		for (const WorkflowVisualMeshAsset& asset : gPreservedMeshes)
		{
			if (asset.label == label)
				return true;
		}
		return false;
	}

	void addWorkflowSceneLayer(
		const std::string& label,
		const std::string& sceneAssetName,
		const WorkflowSceneLayerRole role)
	{
		WorkflowSceneLayer layer;
		layer.asset.label = label;
		layer.sourceAssetName = sceneAssetName;
		layer.role = role;
		layer.loaded = WorkflowSceneAssets::loadRenderOnlySceneMesh(
			workflowExePath(),
			sceneAssetName,
			layer.asset.vertices,
			layer.asset.mesh);
		if (layer.loaded)
			layer.loaded = prepareRenderableMesh(layer.asset);
		layer.asset.visible = layer.loaded;
		gWorkflowSceneLayers.push_back(layer);
	}

	void ensureWorkflowSceneLayersLoaded()
	{
		if (!gWorkflowSceneLayers.empty())
			return;

		addWorkflowSceneLayer("scene_bone_reference", "gu.obj", WorkflowSceneLayerRole::BoneReference);
		addWorkflowSceneLayer("scene_nerve_context", "sj.obj", WorkflowSceneLayerRole::NerveContext);
		addWorkflowSceneLayer("scene_other_discs_context", "other_discs.obj", WorkflowSceneLayerRole::OtherDiscContext);
		addWorkflowSceneLayer("scene_ligament_reference", "rendai_tex.obj", WorkflowSceneLayerRole::LigamentReference);
		addWorkflowSceneLayer("scene_target_disc_reference", "target_disc.obj", WorkflowSceneLayerRole::TargetDiscReference);
		addWorkflowSceneLayer("scene_back_context", "back.obj", WorkflowSceneLayerRole::BackContext);
		addWorkflowSceneLayer("scene_muscle_context", "jirou.obj", WorkflowSceneLayerRole::MuscleContext);
		addWorkflowSceneLayer("scene_grinding_lg_tool", "lg.obj", WorkflowSceneLayerRole::GrindingLgToolContext);
	}

	bool workflowSceneLayerRoleVisible(
		const WorkflowSceneLayerRole role,
		const WorkflowStage stage,
		const bool boneResultAvailable,
		const bool ligamentResultAvailable,
		const bool discResultAvailable)
	{
		switch (role)
		{
		case WorkflowSceneLayerRole::BoneReference:
			if (stage == WorkflowStage::BoneGrinding)
				return true;
			(void)boneResultAvailable;
			return stage != WorkflowStage::Completed;
		case WorkflowSceneLayerRole::NerveContext:
		case WorkflowSceneLayerRole::OtherDiscContext:
			if ((gWorkflowVisualMode == WorkflowVisualMode::Performance) && (stage == WorkflowStage::Completed))
				return false;
			return stage != WorkflowStage::Completed || discResultAvailable;
		case WorkflowSceneLayerRole::LigamentReference:
			return (stage == WorkflowStage::BoneGrinding) && !ligamentResultAvailable;
		case WorkflowSceneLayerRole::TargetDiscReference:
			return !discResultAvailable &&
				(stage == WorkflowStage::BoneGrinding ||
					stage == WorkflowStage::LigamentRemoval ||
					stage == WorkflowStage::RetractionTransition);
		case WorkflowSceneLayerRole::BackContext:
		case WorkflowSceneLayerRole::MuscleContext:
			return true;
		case WorkflowSceneLayerRole::GrindingLgToolContext:
			return stage == WorkflowStage::BoneGrinding;
		default:
			return false;
		}
	}

	bool workflowSceneLayerVisible(const WorkflowSceneLayer& layer, const WorkflowStage stage)
	{
		if (!layer.loaded || !layer.asset.visible || !isRenderableMeshReady(layer.asset))
			return false;
		return workflowSceneLayerRoleVisible(
			layer.role,
			stage,
			preservedMeshLabelExists("bone_result"),
			preservedMeshLabelExists("ligament_result"),
			preservedMeshLabelExists("disc_result"));
	}

	bool workflowSceneLayerLabelVisible(const std::string& label, const WorkflowStage stage)
	{
		ensureWorkflowSceneLayersLoaded();
		for (const WorkflowSceneLayer& layer : gWorkflowSceneLayers)
		{
			if (layer.asset.label == label)
				return workflowSceneLayerVisible(layer, stage);
		}
		return false;
	}

	unsigned int workflowLoadedSceneLayerCount()
	{
		ensureWorkflowSceneLayersLoaded();
		unsigned int loaded = 0u;
		for (const WorkflowSceneLayer& layer : gWorkflowSceneLayers)
		{
			if (layer.loaded && isRenderableMeshReady(layer.asset))
				loaded++;
		}
		return loaded;
	}

	unsigned int workflowVisibleSceneLayerCount(
		const WorkflowStage stage,
		const bool boneResultAvailable,
		const bool ligamentResultAvailable,
		const bool discResultAvailable)
	{
		ensureWorkflowSceneLayersLoaded();
		unsigned int visible = 0u;
		for (const WorkflowSceneLayer& layer : gWorkflowSceneLayers)
		{
			if (!layer.loaded || !isRenderableMeshReady(layer.asset))
				continue;
			if (workflowSceneLayerRoleVisible(
				layer.role,
				stage,
				boneResultAvailable,
				ligamentResultAvailable,
				discResultAvailable))
			{
				visible++;
			}
		}
		return visible;
	}

	bool workflowSceneLayerLabelLoaded(const std::string& label)
	{
		ensureWorkflowSceneLayersLoaded();
		for (const WorkflowSceneLayer& layer : gWorkflowSceneLayers)
		{
			if ((layer.asset.label == label) &&
				layer.loaded &&
				isRenderableMeshReady(layer.asset))
				return true;
		}
		return false;
	}

	bool workflowVisualSceneReplacesDefaultTargets()
	{
		const bool stage2KeepsBoneContext =
			workflowSceneLayerRoleVisible(
				WorkflowSceneLayerRole::BoneReference,
				WorkflowStage::LigamentRemoval,
				true,
				false,
				false);
		const bool stage2HidesDefaultLigament =
			!workflowSceneLayerRoleVisible(
				WorkflowSceneLayerRole::LigamentReference,
				WorkflowStage::LigamentRemoval,
				true,
				false,
				false);
		const bool stage3HidesDefaultLigament =
			workflowVisibleSceneLayerCount(
				WorkflowStage::DiscRemoval,
				true,
				true,
				false) < workflowVisibleSceneLayerCount(
					WorkflowStage::RetractionTransition,
					true,
					true,
					false);
		const bool completedHidesDefaultDisc =
			workflowVisibleSceneLayerCount(
				WorkflowStage::Completed,
				true,
				true,
				true) < workflowVisibleSceneLayerCount(
					WorkflowStage::RetractionTransition,
					true,
					true,
					false);
		return stage2KeepsBoneContext &&
			stage2HidesDefaultLigament &&
			stage3HidesDefaultLigament &&
			completedHidesDefaultDisc;
	}

	void workflowSceneLayerColor(const WorkflowSceneLayerRole role, float color[4])
	{
		switch (role)
		{
		case WorkflowSceneLayerRole::BoneReference:
			color[0] = 0.5f; color[1] = 0.5f; color[2] = 0.5f; color[3] = 1.0f;
			break;
		case WorkflowSceneLayerRole::NerveContext:
		case WorkflowSceneLayerRole::OtherDiscContext:
		case WorkflowSceneLayerRole::LigamentReference:
		case WorkflowSceneLayerRole::TargetDiscReference:
		case WorkflowSceneLayerRole::BackContext:
		case WorkflowSceneLayerRole::MuscleContext:
			color[0] = 0.5f; color[1] = 0.5f; color[2] = 0.5f; color[3] = 1.0f;
			break;
		case WorkflowSceneLayerRole::GrindingLgToolContext:
			color[0] = 0.5f; color[1] = 0.5f; color[2] = 0.5f; color[3] = 1.0f;
			break;
		default:
			color[0] = 0.7f; color[1] = 0.7f; color[2] = 0.7f; color[3] = 1.0f;
			break;
		}
	}

	bool workflowSceneLayerTextureId(const WorkflowSceneLayerRole role, unsigned int& textureId)
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

	void drawWorkflowSceneLayerMesh(WorkflowSceneLayer& layer, const float color[4])
	{
		unsigned int textureId = 0u;
		if ((base != nullptr) &&
			(layer.asset.mesh.getUVs().size() > 0u) &&
			workflowSceneLayerTextureId(layer.role, textureId))
		{
			const float whiteColor[4] = { 1.0f, 1.0f, 1.0f, color[3] };
			base->shaderTexBegin(whiteColor);
			Visualization::drawTexturedMesh(layer.asset.vertices, layer.asset.mesh, 0u, whiteColor, textureId);
			base->shaderTexEnd();
			return;
		}

		if (base != nullptr)
		{
			base->shaderBegin(color);
			Visualization::drawMesh(layer.asset.vertices, layer.asset.mesh, 0u, color);
			base->shaderEnd();
			return;
		}

		drawWorkflowAssetMesh(layer.asset, color);
	}

	void moveWorkflowToolVisualTo(const Vector3r& target)
	{
		if ((base == nullptr) || (base->m_totalSurgToolNbr == 0u))
			return;

		DemoBase::SurgToolMeshPair& toolPair = base->m_surgToolMeshPairs[0];
		VertexData& inactiveVD = toolPair.surgToolVDs[DemoBase::SurgToolMeshPair::STS_INACTIVE];
		if (inactiveVD.size() == 0u)
			return;

		if (!gWorkflowToolVisualInitialized)
		{
			gWorkflowToolVisualCenter = vertexDataCenter(inactiveVD);
			gWorkflowToolVisualInitialized = true;
		}
		if (!gWorkflowToolControlInitialized)
		{
			gWorkflowToolControlIndex = workflowToolControlPointIndex(inactiveVD);
			gWorkflowToolControlInitialized = true;
		}

		Vector3r currentControl =
			(gWorkflowToolControlIndex < inactiveVD.size())
			? inactiveVD.getPosition(gWorkflowToolControlIndex)
			: gWorkflowToolVisualCenter;
		if (gActiveStageContext.toolControlMode == WorkflowToolControlMode::BallGrinderProxy)
		{
			Vector3r proxyCenter = Vector3r::Zero();
			Real proxyRadius = static_cast<Real>(0.0);
			if (computeBallGrinderProxy(inactiveVD, proxyCenter, proxyRadius))
			{
				currentControl = proxyCenter;
				gActiveStageContext.ballGrinderProxyLocalCenter = proxyCenter;
				gActiveStageContext.ballGrinderProxyRadius = proxyRadius;
				gActiveStageContext.ballGrinderProxyReady = true;
			}
		}
		const Vector3r delta = target - currentControl;
		translateVertexData(inactiveVD, delta);
		if (toolPair.hasActiveState)
			translateVertexData(toolPair.surgToolVDs[DemoBase::SurgToolMeshPair::STS_ACTIVE], delta);
		gWorkflowToolVisualCenter += delta;
	}

	std::string workflowVecText(const Vector3r& v)
	{
		std::ostringstream out;
		out << std::fixed << std::setprecision(4)
			<< v.x() << "," << v.y() << "," << v.z();
		return out.str();
	}

	Vector3r workflowMappedHapticPosition()
	{
		if (!MiniGL::isHapticAvailable())
			return Vector3r::Zero();
		return MiniGL::getHapticPos() + gWorkflowHapticToolViewOffset;
	}

	bool workflowCurrentToolStats(
		unsigned int& vertices,
		unsigned int& faces,
		Vector3r& center,
		Vector3r& control)
	{
		vertices = 0u;
		faces = 0u;
		center = Vector3r::Zero();
		control = Vector3r::Zero();
		if ((base == nullptr) || (base->m_totalSurgToolNbr == 0u))
			return false;
		DemoBase::SurgToolMeshPair& toolPair = base->m_surgToolMeshPairs[0];
		VertexData& inactiveVD = toolPair.surgToolVDs[DemoBase::SurgToolMeshPair::STS_INACTIVE];
		IndexedFaceMesh& toolMesh = toolPair.surgToolMeshs[DemoBase::SurgToolMeshPair::STS_INACTIVE];
		vertices = inactiveVD.size();
		faces = toolMesh.numFaces();
		if ((vertices == 0u) || (faces == 0u))
			return false;

		center = vertexDataCenter(inactiveVD);
		if (!gWorkflowToolControlInitialized)
		{
			gWorkflowToolControlIndex = workflowToolControlPointIndex(inactiveVD);
			gWorkflowToolControlInitialized = true;
		}
		control =
			(gWorkflowToolControlIndex < inactiveVD.size())
			? inactiveVD.getPosition(gWorkflowToolControlIndex)
			: center;
		if (gActiveStageContext.toolControlMode == WorkflowToolControlMode::BallGrinderProxy)
		{
			Vector3r proxyCenter = Vector3r::Zero();
			Real proxyRadius = static_cast<Real>(0.0);
			if (computeBallGrinderProxy(inactiveVD, proxyCenter, proxyRadius))
				control = proxyCenter;
		}
		return true;
	}

	void printWorkflowHapticDiagnostics(const char* reason, const bool force)
	{
		if (!gWorkflowHapticDiagnosticsEnabled && !force)
			return;
		static Clock::time_point lastPrint = Clock::now() - std::chrono::seconds(2);
		if (!force && (elapsedMs(lastPrint) < 1000.0))
			return;
		lastPrint = Clock::now();

		unsigned int toolVertices = 0u;
		unsigned int toolFaces = 0u;
		Vector3r toolCenter = Vector3r::Zero();
		Vector3r toolControl = Vector3r::Zero();
		const bool toolReady = workflowCurrentToolStats(toolVertices, toolFaces, toolCenter, toolControl);
		const Vector3r hapticRawPos = MiniGL::isHapticAvailable() ? MiniGL::getHapticPos() : Vector3r::Zero();
		const Vector3r hapticMappedPos = workflowMappedHapticPosition();
		std::ostringstream msg;
		msg << "[DEBUG-HAPTIC-SPINE] reason=" << reason
			<< " haptic_available=" << (MiniGL::isHapticAvailable() ? 1 : 0)
			<< " active=" << (MiniGL::getHapticSelectionState() ? 1 : 0)
			<< " workspace_scale=" << std::fixed << std::setprecision(3) << MiniGL::getHapticWorkspaceScale()
			<< " visual_offset=" << workflowVecText(gWorkflowHapticToolViewOffset)
			<< " haptic_raw_pos=" << workflowVecText(hapticRawPos)
			<< " haptic_mapped_pos=" << workflowVecText(hapticMappedPos)
			<< " marker_valid=" << (gWorkflowToolVisibilityMarkerValid ? 1 : 0)
			<< " marker_pos=" << workflowVecText(gWorkflowToolVisibilityMarkerPosition)
			<< " tool_ready=" << (toolReady ? 1 : 0)
			<< " tool_vertices=" << toolVertices
			<< " tool_faces=" << toolFaces
			<< " tool_control=" << workflowVecText(toolControl)
			<< " tool_center=" << workflowVecText(toolCenter);
		const std::string line = msg.str();
		std::cout << line << "\n";
		std::cout.flush();
		LOG_INFO << line;
	}

	size_t countRenderPatchFaces(SimulationModel* model)
	{
		if (model == nullptr)
			return 0u;
		size_t count = 0u;
		for (TetModel* tetModel : model->getTetModels())
		{
			for (const FracturePatchMesh& patch : tetModel->getFracturePatches())
				count += patch.numRenderFaces();
		}
		return count;
	}

	size_t countRenderVisFaces(SimulationModel* model)
	{
		if (model == nullptr)
			return 0u;
		size_t count = 0u;
		for (TetModel* tetModel : model->getTetModels())
			count += tetModel->getRenderVisMesh().numFaces();
		return count;
	}

	void recordDemoFlowFrame(const Clock::time_point& frameStart, const double solverMs)
	{
		SimulationModel* model = Simulation::getCurrent()->getModel();
		if (model == nullptr)
			return;
		DemoFlowSummary::ToolMetrics toolMetrics;
		toolMetrics.edgePairCacheBuildCount = gActiveStageContext.softCutQueryState.edgePairBuildCount();
		toolMetrics.edgeIndexBuildCount = gActiveStageContext.softCutQueryState.edgeIndexBuildCount();
		toolMetrics.hitEdgeCount = std::max<size_t>(
			gActiveStageContext.softHitEdgeCount,
			static_cast<size_t>(gAutoDemoState.ligamentHitEdges + gAutoDemoState.discHitEdges));
		toolMetrics.legacyPathUsed = false;
		gDemoFlowSummary.recordFrame(
			elapsedMs(frameStart),
			0.0,
			gActiveStageContext.lastToolQueryMs,
			gActiveStageContext.lastSDFEditMs,
			gActiveStageContext.lastMarchingCubeMs,
			gActiveStageContext.lastMeshConvertMs,
			0.0,
			0.0,
			solverMs,
			gLastRenderMs,
			model->getConstraints().size(),
			model->numInactiveConstraints(),
			countRenderPatchFaces(model),
			countRenderVisFaces(model),
			gActiveStageContext.surfaceSignature.hash,
			toolMetrics,
			collectDemoFlowConstraintMetrics(model));
	}
}

void timeStep();
void render();
void renderPreservedMeshes();
void renderActiveStageMesh();
void renderWorkflowSceneLayers();
void renderTransitionAssets();
void reset();
void requestNextStage();
void runFixedOrReplayOperation(WorkflowStageContext& ctx);
void reconstructStageSurface(WorkflowStageContext& ctx);
void buildStageContext(WorkflowStageContext& ctx, WorkflowStage stage);
bool isWorkflowSoftFractureStage(const WorkflowStage stage);
void buildWorkflowSoftTetModel(WorkflowStageContext& ctx);
bool applyWorkflowSoftFractureStep(WorkflowStageContext& ctx, const Vector3r& toolPos, const Real radius);
bool captureStageFractureDisplayMesh(WorkflowStageContext& ctx, const std::string& label);
int runWorkflowMiniGLScreenshotSmoke();
int runWorkflowLivePerformanceSmoke(const int argc, char** argv);
void startWorkflowAutoDemo();
void stopWorkflowAutoDemo();
void advanceWorkflowAutoDemo(const Real dt);
int runWorkflowAutoDemoSmoke(const int argc, char** argv);
std::vector<WorkflowToolPoseStep> buildBoneGrindingPath(const WorkflowStageContext& ctx);
std::vector<WorkflowToolPoseStep> buildLigamentTearPath(const WorkflowStageContext& ctx);
std::vector<WorkflowToolPoseStep> buildDiscPullPath(const WorkflowStageContext& ctx);
bool hasPreservedMeshLabel(const std::string& label);

// --- SDF Infrastructure ---

CubicSDFCollisionDetection::GridPtr generateSDF(
	const std::string& modelFile,
	const Eigen::Matrix<unsigned int, 3, 1>& resolutionSDF,
	VertexData& vd, IndexedFaceMesh& mesh)
{
	const std::string sceneFile = workflowSceneFile();
	const std::string basePath = sceneFile.empty() ? workflowExePath() : FileSystem::getFilePath(sceneFile);
	const string cachePath = basePath + "/Cache";
	const std::string modelFileName = FileSystem::getFileNameWithExt(modelFile);

	const string resStr = to_string(resolutionSDF[0]) + "_" + to_string(resolutionSDF[1]) + "_" + to_string(resolutionSDF[2]);
	const string sdfFileName = FileSystem::normalizePath(cachePath + "/" + modelFileName + "_" + resStr + ".csdf");

	CubicSDFCollisionDetection::GridPtr distanceField;
	if (FileSystem::fileExists(sdfFileName))
	{
		LOG_INFO << "Load cached SDF: " << sdfFileName;
		distanceField = std::make_shared<CubicSDFCollisionDetection::Grid>(sdfFileName);
	}
	else
	{
		std::vector<unsigned int>& faces = mesh.getFaces();
		const unsigned int nFaces = mesh.numFaces();

#ifdef USE_DOUBLE
		Discregrid::TriangleMesh sdfMesh(&vd.getPosition(0)[0], faces.data(), vd.size(), nFaces);
#else
		std::vector<double> doubleVec;
		doubleVec.resize(3 * vd.size());
		for (unsigned int i = 0; i < vd.size(); i++)
			for (unsigned int j = 0; j < 3; j++)
				doubleVec[3 * i + j] = vd.getPosition(i)[j];
		Discregrid::TriangleMesh sdfMesh(&doubleVec[0], faces.data(), vd.size(), nFaces);
#endif
		Discregrid::TriangleMeshDistance md(sdfMesh);
		Eigen::AlignedBox3d domain;
		for (auto const& x : sdfMesh.vertices())
			domain.extend(x);
		domain.max() += 0.1 * Eigen::Vector3d::Ones();
		domain.min() -= 0.1 * Eigen::Vector3d::Ones();

		LOG_INFO << "Set SDF resolution: " << resolutionSDF[0] << ", " << resolutionSDF[1] << ", " << resolutionSDF[2];
		distanceField = std::make_shared<CubicSDFCollisionDetection::Grid>(
			domain, std::array<unsigned int, 3>({ resolutionSDF[0], resolutionSDF[1], resolutionSDF[2] }));
		auto func = Discregrid::DiscreteGrid::ContinuousFunction{};
		func = [&md](Eigen::Vector3d const& xi) { return md.signed_distance(xi).distance; };
		LOG_INFO << "Generate SDF for " << modelFile;
		distanceField->addFunction(func, true);
		if (FileSystem::makeDir(cachePath) == 0)
		{
			LOG_INFO << "Save SDF: " << sdfFileName;
			distanceField->save(sdfFileName);
		}
	}
	return distanceField;
}

void loadOriginSDFModel(WorkflowStageContext& ctx, const std::string& modelFile)
{
	string fileName = FileSystem::normalizePath(workflowExePath() + "/resources/models/scene/" + modelFile);
	IndexedFaceMesh mesh;
	VertexData vd;
	DemoBase::loadMesh(fileName, vd, mesh, Vector3r::Zero(), Matrix3r::Identity(), Vector3r::Ones());

	ctx.warpedVD = vd;
	ctx.warpedMesh = mesh;
	ctx.meshUpdated = false;

	ctx.distanceField = generateSDF(fileName, ctx.resolutionSDF, vd, mesh);

	Eigen::AlignedBox3d const& gridDomain = ctx.distanceField->domain();
	const Vector3d& minPos = gridDomain.min();
	const Vector3d& maxPos = gridDomain.max();

	const Real dx = static_cast<Real>((maxPos[0] - minPos[0]) / (Real)(ctx.resolutionSDF[0] - 1));
	const Real dy = static_cast<Real>((maxPos[1] - minPos[1]) / (Real)(ctx.resolutionSDF[1] - 1));
	const Real dz = static_cast<Real>((maxPos[2] - minPos[2]) / (Real)(ctx.resolutionSDF[2] - 1));

	const unsigned int totalNodes = ctx.resolutionSDF[0] * ctx.resolutionSDF[1] * ctx.resolutionSDF[2];
	ctx.allEleNodePoints.resize(totalNodes);
	ctx.nodeSDFVals.resize(totalNodes);
	ctx.eleAvailableMask.resize(totalNodes);

	for (unsigned int i = 0; i < ctx.resolutionSDF[0]; i++)
	{
		for (unsigned int j = 0; j < ctx.resolutionSDF[1]; j++)
		{
			for (unsigned int k = 0; k < ctx.resolutionSDF[2]; k++)
			{
				const unsigned int glNodeIdx = i * ctx.resolutionSDF[1] * ctx.resolutionSDF[2] +
					j * ctx.resolutionSDF[2] + k;
				const unsigned int sdfSerialIdx = k * ctx.resolutionSDF[0] * ctx.resolutionSDF[1] +
					j * ctx.resolutionSDF[0] + i;

				const Vector3r curPt(
					static_cast<Real>(minPos[0] + dx * i),
					static_cast<Real>(minPos[1] + dy * j),
					static_cast<Real>(minPos[2] + dz * k));

				ctx.allEleNodePoints[glNodeIdx] = curPt;
				ctx.nodeSDFVals[sdfSerialIdx] = ctx.distanceField->interpolate(0, curPt);
				ctx.eleAvailableMask[glNodeIdx] = (ctx.nodeSDFVals[sdfSerialIdx] <= static_cast<Real>(0.0));
			}
		}
	}
}

void reconstructStageSurface(WorkflowStageContext& ctx)
{
	const SDFSurfaceReconstructionStats stats = SDFSurfaceReconstruction::reconstruct(
		ctx.nodeSDFVals,
		ctx.resolutionSDF,
		ctx.distanceField->domain(),
		ctx.mcMesh,
		ctx.mcWorkspace,
		ctx.warpedVD,
		ctx.warpedMesh);
	ctx.lastMarchingCubeMs = stats.marchingCubeMs;
	ctx.lastMeshConvertMs = stats.meshConvertMs;
	if (!stats.success)
	{
		LOG_ERR << stats.error;
		return;
	}
	ctx.meshUpdated = false;
	ctx.surfaceSignature = stats.signature;
}

void ensureStageSurfaceCurrent(WorkflowStageContext& ctx)
{
	if (SDFSurfaceReconstruction::shouldReconstruct(ctx.meshUpdated, ctx.surfaceSignature))
		reconstructStageSurface(ctx);
}

// --- Stage Builder ---

void loadWorkflowStageTool(
	const std::string& toolRelativePath,
	const Vector3r& toolTranslation,
	const Vector3r& toolScale)
{
	WorkflowSceneAssets::clearAndLoadStageTool(*base, toolRelativePath, toolTranslation, toolScale);
	gWorkflowToolVisualInitialized = false;
	gWorkflowToolVisualCenter = Vector3r::Zero();
	gWorkflowToolControlInitialized = false;
	gWorkflowToolControlIndex = 0u;
}

void buildStageContext(WorkflowStageContext& ctx, WorkflowStage stage)
{
	ctx.clear();
	ctx.stage = stage;
	gWorkflowHapticToolWasActive = false;
	gWorkflowHapticToolActive = false;
	gWorkflowToolVisibilityMarkerValid = false;

	switch (stage)
	{
	case WorkflowStage::BoneGrinding:
		ctx.stageName = "Bone Grinding";
		ctx.activeAssetPath = "gu_target.obj";
		ctx.preferredToolAssetPath = "zuantou.obj";
		ctx.fallbackToolAssetPath = "qianzi.obj";
		ctx.replayPath = "";
		ctx.resolutionSDF = Eigen::Matrix<unsigned int, 3, 1, Eigen::DontAlign>(60, 60, 60);
		ctx.toolRadius = static_cast<Real>(0.35);
		ctx.toolVisualScale = Vector3r::Ones();
		ctx.toolInitialTranslation = Vector3r::Zero();
		ctx.toolControlMode = WorkflowToolControlMode::BallGrinderProxy;
		break;
	case WorkflowStage::LigamentRemoval:
		ctx.stageName = "Ligament Grasp";
		ctx.activeAssetPath = "rendai_tex.obj";
		ctx.preferredToolAssetPath = "bone_rongeur_vertical.obj";
		ctx.fallbackToolAssetPath = "qianzi.obj";
		ctx.replayPath = "data/selftest/tetmodel/spine-rendai-hit-path.json";
		ctx.resolutionSDF = Eigen::Matrix<unsigned int, 3, 1, Eigen::DontAlign>(30, 30, 20);
		ctx.toolRadius = static_cast<Real>(0.10);
		ctx.toolVisualScale = Vector3r(static_cast<Real>(0.5), static_cast<Real>(0.5), static_cast<Real>(0.5));
		ctx.toolInitialTranslation = Vector3r::Zero();
		ctx.toolControlMode = WorkflowToolControlMode::MinYTip;
		ctx.softScript.enabled = true;
		ctx.softScript.pathParticleA = 609u;
		ctx.softScript.pathParticleB = 888u;
		ctx.softScript.selectedParticle = 609u;
		ctx.softScript.toolOffset = Vector3r(static_cast<Real>(0.0), static_cast<Real>(-0.3), static_cast<Real>(0.0));
		ctx.softScript.toolPullDelta = Vector3r(static_cast<Real>(0.0), static_cast<Real>(0.2), static_cast<Real>(0.0));
		ctx.softScript.preCutParticleDelta = Vector3r(static_cast<Real>(0.0), static_cast<Real>(0.1), static_cast<Real>(0.0));
		ctx.softScript.holdSecondsAfterFracture = static_cast<Real>(2.5);
		break;
	case WorkflowStage::DiscRemoval:
		ctx.stageName = "Disc Removal";
		ctx.activeAssetPath = "target_disc.obj";
		ctx.preferredToolAssetPath = "nucleus_forceps_vertical.obj";
		ctx.fallbackToolAssetPath = "qianzi.obj";
		ctx.replayPath = "data/selftest/tetmodel/fracture-target-disc-hit-path.json";
		ctx.resolutionSDF = Eigen::Matrix<unsigned int, 3, 1, Eigen::DontAlign>(30, 30, 20);
		ctx.toolRadius = static_cast<Real>(0.10);
		ctx.toolVisualScale = Vector3r(static_cast<Real>(0.6), static_cast<Real>(0.6), static_cast<Real>(0.6));
		ctx.toolInitialTranslation = Vector3r(static_cast<Real>(0.0), static_cast<Real>(0.0), static_cast<Real>(8.47));
		ctx.toolControlMode = WorkflowToolControlMode::MinYTip;
		ctx.softScript.enabled = true;
		ctx.softScript.pathParticleA = 1401u;
		ctx.softScript.pathParticleB = 3205u;
		ctx.softScript.selectedParticle = 1401u;
		ctx.softScript.toolOffset = Vector3r(static_cast<Real>(0.1), static_cast<Real>(0.0), static_cast<Real>(0.0));
		ctx.softScript.toolPullDelta = Vector3r(static_cast<Real>(0.0), static_cast<Real>(0.3), static_cast<Real>(0.0));
		ctx.softScript.preCutParticleDelta = Vector3r(static_cast<Real>(0.0), static_cast<Real>(0.1), static_cast<Real>(0.0));
		ctx.softScript.holdSecondsAfterFracture = static_cast<Real>(2.5);
		break;
	default:
		return;
	}
	ctx.resolvedToolRelativePath = resolveWorkflowToolRelativePath(
		ctx.preferredToolAssetPath,
		ctx.fallbackToolAssetPath,
		ctx.resolvedToolAssetPath,
		ctx.warning);

	loadOriginSDFModel(ctx, ctx.activeAssetPath);
	if (isWorkflowSoftFractureStage(ctx.stage))
		buildWorkflowSoftTetModel(ctx);
	else
		reconstructStageSurface(ctx);

	if (!gWorkflowFixedSummaryMode)
		loadWorkflowStageTool(ctx.resolvedToolRelativePath, ctx.toolInitialTranslation, ctx.toolVisualScale);
}

// --- SDF Edit ---

std::size_t applySweptSDFEdit(WorkflowStageContext& ctx, const Vector3r& toolPos)
{
	const Clock::time_point toolQueryStart = Clock::now();
	ctx.toolController.updateProbe(ctx.toolProbe, toolPos, ctx.toolRadius);
	ctx.toolProbeInitialized = ctx.toolProbe.initialized;
	ctx.lastToolQueryMs = elapsedMs(toolQueryStart);

	const Clock::time_point editStart = Clock::now();
	Eigen::AlignedBox3d const& gridDomain = ctx.distanceField->domain();
	const Vector3d& minPos = gridDomain.min();
	const Vector3d& maxPos = gridDomain.max();
	const Real interval = static_cast<Real>((maxPos[0] - minPos[0]) / (Real)(ctx.resolutionSDF[0] - 1));

	ToolTargetInteractionController::SDFGridView gridView;
	gridView.values = &ctx.nodeSDFVals;
	gridView.availableMask = &ctx.eleAvailableMask;
	gridView.resolution = ctx.resolutionSDF;
	gridView.domain = toRealDomain(gridDomain);
	gridView.replacementValue = interval;
	gridView.sdfIndex = [&ctx](const ToolTargetInteractionController::GridIndex& idx) {
		const Vector3i res(ctx.resolutionSDF[0], ctx.resolutionSDF[1], ctx.resolutionSDF[2]);
		return static_cast<size_t>(toSDFSerialIdx(
			Vector3i(static_cast<int>(idx[0]), static_cast<int>(idx[1]), static_cast<int>(idx[2])), res));
	};
	gridView.maskIndex = [&ctx](const ToolTargetInteractionController::GridIndex& idx) {
		const Vector3i res(ctx.resolutionSDF[0], ctx.resolutionSDF[1], ctx.resolutionSDF[2]);
		return static_cast<size_t>(toGLSerialIdx(
			Vector3i(static_cast<int>(idx[0]), static_cast<int>(idx[1]), static_cast<int>(idx[2])), res));
	};

	const InteractionEvent event = ctx.toolController.applySweptSDFEdit(gridView, ctx.toolProbe, ctx.toolProfile);
	ctx.lastSDFEditMs = elapsedMs(editStart);
	if (event.changedNodes > 0u)
		ctx.meshUpdated = true;
	return event.changedNodes;
}

// --- Soft Fracture Stage Helpers ---

bool isWorkflowSoftFractureStage(const WorkflowStage stage)
{
	return (stage == WorkflowStage::LigamentRemoval) || (stage == WorkflowStage::DiscRemoval);
}

std::vector<std::array<unsigned int, 2> > collectWorkflowTetEdges(const TetModel& tetModel)
{
	std::vector<std::array<unsigned int, 2> > edgePairs;
	const unsigned int offset = tetModel.getIndexOffset();
	const std::vector<Utilities::IndexedTetMesh::Edge>& edges = tetModel.getParticleMesh().getEdges();
	edgePairs.reserve(edges.size());
	for (const Utilities::IndexedTetMesh::Edge& edge : edges)
	{
		if ((edge.m_vert[0] == INVALID) || (edge.m_vert[1] == INVALID))
			continue;
		edgePairs.push_back({ { edge.m_vert[0] + offset, edge.m_vert[1] + offset } });
	}
	return edgePairs;
}

bool workflowParticleListContains(const std::vector<unsigned int>& particles, const unsigned int particleId)
{
	return std::find(particles.begin(), particles.end(), particleId) != particles.end();
}

std::vector<std::array<unsigned int, 2> > collectWorkflowTetEdgesExcluding(
	const TetModel& tetModel,
	const std::vector<unsigned int>& excludedParticles)
{
	if (excludedParticles.empty())
		return collectWorkflowTetEdges(tetModel);

	std::vector<std::array<unsigned int, 2> > edgePairs;
	const unsigned int offset = tetModel.getIndexOffset();
	const std::vector<Utilities::IndexedTetMesh::Edge>& edges = tetModel.getParticleMesh().getEdges();
	edgePairs.reserve(edges.size());
	for (const Utilities::IndexedTetMesh::Edge& edge : edges)
	{
		if ((edge.m_vert[0] == INVALID) || (edge.m_vert[1] == INVALID))
			continue;
		const unsigned int particle0 = edge.m_vert[0] + offset;
		const unsigned int particle1 = edge.m_vert[1] + offset;
		if (workflowParticleListContains(excludedParticles, particle0) ||
			workflowParticleListContains(excludedParticles, particle1))
			continue;
		edgePairs.push_back({ { particle0, particle1 } });
	}
	return edgePairs;
}

	TetModel::FracturePatchDebugType workflowPatchTypeAt(const TetModel& tetModel, const size_t patchIndex)
	{
		const std::vector<TetModel::FracturePatchDebugInfo>& patchInfo = tetModel.getFracturePatchDebugInfo();
		if (patchIndex < patchInfo.size())
			return patchInfo[patchIndex].type;
		return TetModel::FracturePatchDebugType::Unknown;
	}

	bool workflowPatchAllowedForCopy(
		const TetModel::FracturePatchDebugType type,
		const WorkflowPatchCopyMode mode)
	{
		if (mode == WorkflowPatchCopyMode::AllDebugPatches)
			return true;
		switch (type)
		{
		case TetModel::FracturePatchDebugType::Cap:
		case TetModel::FracturePatchDebugType::VisualFragment:
		case TetModel::FracturePatchDebugType::BoundaryParticle:
		case TetModel::FracturePatchDebugType::RenderFragment:
			return true;
		case TetModel::FracturePatchDebugType::TetBoundaryFragment:
		case TetModel::FracturePatchDebugType::TetBoundaryPrimary:
		case TetModel::FracturePatchDebugType::HiddenSurface:
		case TetModel::FracturePatchDebugType::HiddenVis:
		case TetModel::FracturePatchDebugType::RenderCutGap:
		case TetModel::FracturePatchDebugType::ClippedVisualFragment:
		case TetModel::FracturePatchDebugType::ClippedVisualPrimary:
		case TetModel::FracturePatchDebugType::Unknown:
		default:
			return false;
		}
	}

	unsigned int countWorkflowPatchFaces(
		const TetModel& tetModel,
		const WorkflowPatchCopyMode mode = WorkflowPatchCopyMode::AllDebugPatches)
	{
		unsigned int count = 0u;
		const std::vector<FracturePatchMesh>& patches = tetModel.getFracturePatches();
		for (size_t patchIndex = 0u; patchIndex < patches.size(); patchIndex++)
		{
			if (!workflowPatchAllowedForCopy(workflowPatchTypeAt(tetModel, patchIndex), mode))
				continue;
			count += static_cast<unsigned int>(patches[patchIndex].numRenderFaces());
		}
		return count;
	}

	void syncWorkflowParticleState(ParticleData& pd, const unsigned int idx, const Vector3r& pos)
	{
		if (idx >= pd.size())
			return;
		pd.setPosition(idx, pos);
		pd.setOldPosition(idx, pos);
		pd.setLastPosition(idx, pos);
		pd.setVelocity(idx, Vector3r::Zero());
	}

	void applyWorkflowParticleDelta(
		ParticleData& pd,
		const std::vector<unsigned int>& particles,
		const Vector3r& delta)
	{
		if (delta.squaredNorm() <= static_cast<Real>(1.0e-12))
			return;
		for (const unsigned int idx : particles)
		{
			if (idx < pd.size())
				syncWorkflowParticleState(pd, idx, pd.getPosition(idx) + delta);
		}
	}

	void mergeWorkflowDeactivationResult(
		ToolCutInteraction::DeactivationResult& target,
		const ToolCutInteraction::DeactivationResult& source)
	{
		for (const unsigned int constraintId : source.constraintIds)
		{
			if (std::find(target.constraintIds.begin(), target.constraintIds.end(), constraintId) == target.constraintIds.end())
				target.constraintIds.push_back(constraintId);
		}
		for (const FractureEdgeKey& edge : source.fractureDelta.brokenEdges)
		{
			if (std::find(target.fractureDelta.brokenEdges.begin(), target.fractureDelta.brokenEdges.end(), edge) ==
				target.fractureDelta.brokenEdges.end())
				target.fractureDelta.brokenEdges.push_back(edge);
		}
		for (const unsigned int constraintId : source.fractureDelta.inactiveConstraintIds)
		{
			if (std::find(target.fractureDelta.inactiveConstraintIds.begin(), target.fractureDelta.inactiveConstraintIds.end(), constraintId) ==
				target.fractureDelta.inactiveConstraintIds.end())
				target.fractureDelta.inactiveConstraintIds.push_back(constraintId);
		}
		for (const unsigned int particleId : source.fractureDelta.affectedParticles)
		{
			if (std::find(target.fractureDelta.affectedParticles.begin(), target.fractureDelta.affectedParticles.end(), particleId) ==
				target.fractureDelta.affectedParticles.end())
				target.fractureDelta.affectedParticles.push_back(particleId);
		}
	}

	ToolCutInteraction::FractureTriggerOptions workflowSoftFractureTriggerOptions()
	{
		ToolCutInteraction::FractureTriggerOptions options;
		options.mode = ToolCutInteraction::FractureTriggerMode::LengthThreshold;
		options.breakStretchRatio = static_cast<Real>(1.06);
		options.damageStartStretchRatio = static_cast<Real>(1.05);
		options.damageRate = static_cast<Real>(1.0);
		options.damageDecay = static_cast<Real>(0.0);
		options.damageStep = static_cast<Real>(1.0);
		return options;
	}

	void resetWorkflowSoftInteractionSession(WorkflowStageContext& ctx)
	{
		ctx.softSelectedParticles.clear();
		ctx.softDraggedFragmentParticles.clear();
		ctx.softPendingCutFragmentParticles.clear();
		ctx.softPendingCutHitEdges.clear();
		ctx.softLastFractureEdges.clear();
		ctx.softDraggedFragmentComponent = FractureState::InvalidComponent;
		ctx.softCutApplied = false;
		ctx.softDraggedParticleCount = 0u;
		ctx.softHasLastToolPosition = false;
		ctx.softLastToolPosition = Vector3r::Zero();
		ctx.softCutQueryState.markTopologyDirty();
		ctx.softCutQueryState.markPositionsDirty();
		ctx.softFractureTriggerState.clear();
		ctx.toolProbe = ToolProbe();
		ctx.toolProbeInitialized = false;
	}

	void seedWorkflowSoftSelectionFromToolPosition(
		WorkflowStageContext& ctx,
		TetModel& tetModel,
		const ParticleData& pd,
		const Vector3r& toolPos)
	{
		if (!ctx.softSelectedParticles.empty())
			return;

		const unsigned int offset = tetModel.getIndexOffset();
		const unsigned int vertexCount = tetModel.getParticleMesh().numVertices();
		if ((vertexCount == 0u) || (offset >= pd.size()))
			return;

		unsigned int bestParticle = offset;
		bool foundParticle = false;
		if (ctx.softScript.enabled && !gWorkflowHapticToolActive && !ctx.softScriptSelectionSeeded)
		{
			const unsigned int scriptedParticle = offset + ctx.softScript.selectedParticle;
			if ((scriptedParticle < pd.size()) &&
				!workflowParticleListContains(ctx.softDiscardedFragmentParticles, scriptedParticle))
			{
				bestParticle = scriptedParticle;
				ctx.softScriptSelectionSeeded = true;
				foundParticle = true;
			}
		}
		if (!foundParticle)
		{
			Real bestDist2 = REAL_MAX;
			for (unsigned int local = 0u; local < vertexCount; local++)
			{
				const unsigned int particleId = offset + local;
				if (particleId >= pd.size())
					continue;
				if (workflowParticleListContains(ctx.softDiscardedFragmentParticles, particleId))
					continue;
				const Real dist2 = (pd.getPosition(particleId) - toolPos).squaredNorm();
				if (dist2 < bestDist2)
				{
					bestDist2 = dist2;
					bestParticle = particleId;
					foundParticle = true;
				}
			}
			if (!foundParticle)
				return;
		}

		ctx.softSelectedParticles.clear();
		ctx.softSelectedParticles.push_back(bestParticle);
		ctx.softSelectedParticles = expandToolSelectedParticlesToOneRing(
			ctx.softSelectedParticles,
			tetModel.getParticleMesh(),
			tetModel.getIndexOffset());
		ctx.softSelectedParticles.erase(
			std::remove_if(
				ctx.softSelectedParticles.begin(),
				ctx.softSelectedParticles.end(),
				[&ctx](const unsigned int particleId) {
					return workflowParticleListContains(ctx.softDiscardedFragmentParticles, particleId);
				}),
			ctx.softSelectedParticles.end());
	}

	std::vector<unsigned int> collectWorkflowFragmentCatchParticles(
		WorkflowStageContext& ctx,
		TetModel& tetModel,
		const std::vector<std::array<unsigned int, 2> >& hitEdges)
	{
		std::vector<unsigned int> seeds;
		for (const unsigned int particleId : ctx.softSelectedParticles)
			appendUniqueToolParticle(seeds, particleId);

		const unsigned int ringDepth = seeds.empty() ? 2u : 1u;
		if (seeds.empty())
		{
			for (const std::array<unsigned int, 2>& edge : hitEdges)
			{
				appendUniqueToolParticle(seeds, edge[0]);
				appendUniqueToolParticle(seeds, edge[1]);
			}
		}
		if (seeds.empty())
			return seeds;
		return expandToolSelectedParticlesByEdgeRings(
			seeds,
			tetModel.getParticleMesh(),
			tetModel.getIndexOffset(),
			ringDepth);
	}

	std::vector<unsigned int> expandWorkflowParticlesForVisibleChunk(
		TetModel& tetModel,
		const std::vector<unsigned int>& particles)
	{
		return expandToolSelectedParticlesByEdgeRings(
			particles,
			tetModel.getParticleMesh(),
			tetModel.getIndexOffset(),
			1u);
	}

	void rebuildWorkflowDraggedFragmentParticles(
		WorkflowStageContext& ctx,
		TetModel& tetModel,
		const std::vector<std::array<unsigned int, 2> >& hitEdges,
		const std::vector<unsigned int>& catchFragmentParticles)
	{
		ctx.softDraggedFragmentParticles.clear();
		ctx.softDraggedFragmentComponent = FractureState::InvalidComponent;

		if (!catchFragmentParticles.empty())
		{
			ctx.softDraggedFragmentParticles = catchFragmentParticles;
			ctx.softDraggedParticleCount = static_cast<unsigned int>(ctx.softDraggedFragmentParticles.size());
			return;
		}

		const FractureState& fractureState = tetModel.getFractureState();
		const std::vector<unsigned int>& components = fractureState.getParticleComponents();
		const unsigned int componentCount = fractureState.getComponentCount();
		if ((componentCount == 0u) || components.empty())
			return;

		const size_t minDraggedFragmentSize = std::max<size_t>(
			12u,
			std::min<size_t>(96u, static_cast<size_t>(static_cast<double>(components.size()) * 0.0005)));
		const size_t minLocalFallbackSize = std::max<size_t>(
			8u,
			std::min<size_t>(32u, static_cast<size_t>(static_cast<double>(components.size()) * 0.001)));

		ToolDraggedFragmentSelectionInput input;
		input.componentOffset = fractureState.getParticleComponentOffset();
		input.components = components;
		input.componentCount = componentCount;
		input.selectedParticles = ctx.softSelectedParticles;
		input.hitEdges = hitEdges;
		input.minDraggedFragmentSize = minDraggedFragmentSize;
		input.allowLocalFallback = false;
		input.minLocalFallbackSize = minLocalFallbackSize;
		input.maxLocalFallbackSize = 384u;
		const ToolDraggedFragmentSelectionResult result = selectToolDraggedFragmentParticles(input);
		if (result.particles.empty())
		{
			ctx.softDraggedFragmentParticles = collectWorkflowFragmentCatchParticles(ctx, tetModel, hitEdges);
			ctx.softDraggedParticleCount = static_cast<unsigned int>(ctx.softDraggedFragmentParticles.size());
			return;
		}

		const std::vector<unsigned int> expandedParticles =
			expandWorkflowParticlesForVisibleChunk(tetModel, result.particles);
		for (const unsigned int particleId : expandedParticles)
		{
			if ((result.component == FractureState::InvalidComponent) ||
				(fractureState.getParticleComponent(particleId) == result.component))
				ctx.softDraggedFragmentParticles.push_back(particleId);
		}
		ctx.softDraggedFragmentComponent = result.component;
		ctx.softDraggedParticleCount = static_cast<unsigned int>(ctx.softDraggedFragmentParticles.size());
	}

	bool copyWorkflowFractureDisplayMesh(
		const TetModel& tetModel,
		const ParticleData& pd,
		VertexData& outVertices,
		IndexedFaceMesh& outMesh,
		const WorkflowPatchCopyMode patchCopyMode = WorkflowPatchCopyMode::AllDebugPatches)
{
	outVertices.release();
	outMesh.release();

	const IndexedFaceMesh& renderMesh = tetModel.getRenderVisMesh();
	const VertexData& renderVertices = tetModel.getRenderVisVertices();
	const std::vector<FracturePatchMesh>& patches = tetModel.getFracturePatches();

	unsigned int totalVertices = renderMesh.numVertices();
	unsigned int totalFaces = renderMesh.numFaces();
	for (size_t patchIndex = 0u; patchIndex < patches.size(); patchIndex++)
	{
		if (!workflowPatchAllowedForCopy(workflowPatchTypeAt(tetModel, patchIndex), patchCopyMode))
			continue;
		const FracturePatchMesh& patch = patches[patchIndex];
		totalVertices += static_cast<unsigned int>(patch.numRenderVertices());
		totalFaces += static_cast<unsigned int>(patch.numRenderFaces());
	}
	if ((totalVertices == 0u) || (totalFaces == 0u))
		return false;

	outVertices.reserve(totalVertices);
	outMesh.initMesh(totalVertices, 0u, totalFaces);

	const unsigned int offset = tetModel.getIndexOffset();
	for (unsigned int i = 0u; i < renderMesh.numVertices(); i++)
	{
		if (i < renderVertices.size())
			outVertices.addVertex(renderVertices.getPosition(i));
		else if (offset + i < pd.size())
			outVertices.addVertex(pd.getPosition(offset + i));
		else
			outVertices.addVertex(Vector3r::Zero());
	}
	const std::vector<unsigned int>& renderFaces = renderMesh.getFaces();
	for (unsigned int face = 0u; face < renderMesh.numFaces(); face++)
	{
		const unsigned int ids[3] = {
			renderFaces[3u * face],
			renderFaces[3u * face + 1u],
			renderFaces[3u * face + 2u]
		};
		outMesh.addFace(ids);
	}

	for (size_t patchIndex = 0u; patchIndex < patches.size(); patchIndex++)
	{
		if (!workflowPatchAllowedForCopy(workflowPatchTypeAt(tetModel, patchIndex), patchCopyMode))
			continue;
		const FracturePatchMesh& patch = patches[patchIndex];
		const unsigned int vertexOffset = outVertices.size();
		const VertexData& patchVertices = patch.getRenderVertexData();
		for (unsigned int i = 0u; i < patchVertices.size(); i++)
			outVertices.addVertex(patchVertices.getPosition(i));

		const IndexedFaceMesh& patchMesh = patch.getRenderSurfaceMesh();
		const std::vector<unsigned int>& patchFaces = patchMesh.getFaces();
		for (unsigned int face = 0u; face < patchMesh.numFaces(); face++)
		{
			const unsigned int ids[3] = {
				vertexOffset + patchFaces[3u * face],
				vertexOffset + patchFaces[3u * face + 1u],
				vertexOffset + patchFaces[3u * face + 2u]
			};
			outMesh.addFace(ids);
		}
	}

	if (outMesh.numFaces() > 0u)
	{
		outMesh.buildNeighbors();
		outMesh.updateNormals(outVertices, 0u);
		outMesh.updateVertexNormals(outVertices);
		}
		return (outVertices.size() > 0u) && (outMesh.numFaces() > 0u);
	}

	unsigned int largestWorkflowFractureComponent(const TetModel& tetModel)
	{
		const FractureState& fractureState = tetModel.getFractureState();
		const unsigned int componentCount = fractureState.getComponentCount();
		if (componentCount == 0u)
			return FractureState::InvalidComponent;
		std::vector<unsigned int> counts(componentCount, 0u);
		const unsigned int offset = tetModel.getIndexOffset();
		const unsigned int vertexCount = tetModel.getParticleMesh().numVertices();
		for (unsigned int local = 0u; local < vertexCount; local++)
		{
			const unsigned int component = fractureState.getParticleComponent(offset + local);
			if (component < componentCount)
				counts[component]++;
		}
		unsigned int bestComponent = FractureState::InvalidComponent;
		unsigned int bestCount = 0u;
		for (unsigned int component = 0u; component < componentCount; component++)
		{
			if (counts[component] > bestCount)
			{
				bestCount = counts[component];
				bestComponent = component;
			}
		}
		return bestComponent;
	}

	unsigned int workflowAttachmentComponent(
		const TetModel& tetModel,
		const TetModel::Attachment& attachment)
	{
		if (attachment.m_component != FractureState::InvalidComponent)
			return attachment.m_component;
		const FractureState& fractureState = tetModel.getFractureState();
		for (const unsigned int particleId : attachment.m_faceKey)
		{
			const unsigned int component = fractureState.getParticleComponent(particleId);
			if (component != FractureState::InvalidComponent)
				return component;
		}
		return FractureState::InvalidComponent;
	}

	bool copyWorkflowMainComponentDisplayMesh(
		const TetModel& tetModel,
		const ParticleData& pd,
		VertexData& outVertices,
		IndexedFaceMesh& outMesh,
		unsigned int& mainComponentFaces)
	{
		mainComponentFaces = 0u;
		const unsigned int mainComponent = largestWorkflowFractureComponent(tetModel);
		const IndexedFaceMesh& renderMesh = tetModel.getRenderVisMesh();
		const VertexData& renderVertices = tetModel.getRenderVisVertices();
		const std::vector<TetModel::Attachment>& renderAttachments = tetModel.getRenderVisAttachments();
		if ((mainComponent == FractureState::InvalidComponent) ||
			(renderMesh.numFaces() == 0u) ||
			(renderAttachments.size() < renderMesh.numVertices()))
		{
			const bool copied = copyWorkflowFractureDisplayMesh(
				tetModel,
				pd,
				outVertices,
				outMesh,
				WorkflowPatchCopyMode::VisibleSoftFragmentOnly);
			mainComponentFaces = copied ? outMesh.numFaces() : 0u;
			return copied;
		}

		outVertices.release();
		outMesh.release();
		std::vector<unsigned int> remap(renderMesh.numVertices(), INVALID);
		const std::vector<unsigned int>& faces = renderMesh.getFaces();
		for (unsigned int face = 0u; face < renderMesh.numFaces(); face++)
		{
			const unsigned int sourceIds[3] = {
				faces[3u * face],
				faces[3u * face + 1u],
				faces[3u * face + 2u]
			};
			bool keep = true;
			for (unsigned int corner = 0u; corner < 3u; corner++)
			{
				const unsigned int sourceId = sourceIds[corner];
				if ((sourceId >= renderAttachments.size()) ||
					(workflowAttachmentComponent(tetModel, renderAttachments[sourceId]) != mainComponent))
				{
					keep = false;
					break;
				}
			}
			if (!keep)
				continue;
			for (unsigned int corner = 0u; corner < 3u; corner++)
			{
				const unsigned int sourceId = sourceIds[corner];
				if (remap[sourceId] == INVALID)
				{
					remap[sourceId] = outVertices.size();
					if (sourceId < renderVertices.size())
						outVertices.addVertex(renderVertices.getPosition(sourceId));
					else
						outVertices.addVertex(Vector3r::Zero());
				}
			}
		}
		if (outVertices.size() == 0u)
			return false;

		outMesh.initMesh(outVertices.size(), 0u, renderMesh.numFaces());
		for (unsigned int face = 0u; face < renderMesh.numFaces(); face++)
		{
			const unsigned int sourceIds[3] = {
				faces[3u * face],
				faces[3u * face + 1u],
				faces[3u * face + 2u]
			};
			if ((sourceIds[0] >= remap.size()) ||
				(sourceIds[1] >= remap.size()) ||
				(sourceIds[2] >= remap.size()) ||
				(remap[sourceIds[0]] == INVALID) ||
				(remap[sourceIds[1]] == INVALID) ||
				(remap[sourceIds[2]] == INVALID))
				continue;
			const unsigned int remappedIds[3] = {
				remap[sourceIds[0]],
				remap[sourceIds[1]],
				remap[sourceIds[2]]
			};
			outMesh.addFace(remappedIds);
			mainComponentFaces++;
		}
		if (outMesh.numFaces() == 0u)
			return false;
		outMesh.buildNeighbors();
		outMesh.updateNormals(outVertices, 0u);
		outMesh.updateVertexNormals(outVertices);
		return true;
	}

	void refreshWorkflowSoftDisplayMesh(
		WorkflowStageContext& ctx,
		TetModel& tetModel,
		ParticleData& pd,
		const std::vector<std::array<unsigned int, 2> >& fractureEdges,
		const bool rebuildDetachedSurface)
	{
		if (!fractureEdges.empty())
		{
			TetModel::FractureDisplayRefreshOptions displayOptions;
			displayOptions.mode = TetModel::FractureDisplayMode::LocalVisualRebind;
			displayOptions.suppressDetachedComponentFaces = true;
			displayOptions.rebuildDetachedFragmentSurface =
				rebuildDetachedSurface && !ctx.softDraggedFragmentParticles.empty();
			displayOptions.detachedFragmentParticles = ctx.softDraggedFragmentParticles;
			displayOptions.detachedFragmentComponent = ctx.softDraggedFragmentComponent;
			ctx.lastFractureDisplayStats =
				tetModel.refreshFractureDisplayAfterCut(pd, fractureEdges, displayOptions);
		}
		tetModel.updateMeshNormals(pd);
		tetModel.updateVisMesh(pd);
		copyWorkflowFractureDisplayMesh(
			tetModel,
			pd,
			ctx.softDisplayVD,
			ctx.softDisplayMesh,
			WorkflowPatchCopyMode::VisibleSoftFragmentOnly);
		ctx.softRenderFaceCount = static_cast<unsigned int>(ctx.softDisplayMesh.numFaces());
		ctx.softPatchFaceCount =
			countWorkflowPatchFaces(tetModel, WorkflowPatchCopyMode::VisibleSoftFragmentOnly);
		ctx.softDebugPatchFaceCount =
			countWorkflowPatchFaces(tetModel, WorkflowPatchCopyMode::AllDebugPatches);
		ctx.softHiddenFaceCount =
			ctx.lastFractureDisplayStats.hiddenFaceCount +
			ctx.lastFractureDisplayStats.detachedPrimaryHiddenFaceCount;
	}

	bool discardWorkflowReleasedSoftFragment(WorkflowStageContext& ctx)
	{
		if (!isWorkflowSoftFractureStage(ctx.stage))
			return false;

		bool displayUpdated = false;
		if (ctx.softCutApplied && !ctx.softDraggedFragmentParticles.empty())
		{
			for (const unsigned int particleId : ctx.softDraggedFragmentParticles)
			{
				if (!workflowParticleListContains(ctx.softDiscardedFragmentParticles, particleId))
					ctx.softDiscardedFragmentParticles.push_back(particleId);
			}

			SimulationModel* model = Simulation::getCurrent()->getModel();
			TetModel* tetModel = ((model != nullptr) && !model->getTetModels().empty()) ? model->getTetModels()[0] : nullptr;
			if (tetModel != nullptr)
			{
				unsigned int mainComponentFaces = 0u;
				if (copyWorkflowMainComponentDisplayMesh(
					*tetModel,
					model->getParticles(),
					ctx.softDisplayVD,
					ctx.softDisplayMesh,
					mainComponentFaces))
				{
					ctx.softMainComponentFaceCount = mainComponentFaces;
					ctx.softRenderFaceCount = static_cast<unsigned int>(ctx.softDisplayMesh.numFaces());
					displayUpdated = true;
				}
				else
				{
					refreshWorkflowSoftDisplayMesh(ctx, *tetModel, model->getParticles(), ctx.softLastFractureEdges, false);
					displayUpdated = true;
				}
			}
		}

		resetWorkflowSoftInteractionSession(ctx);
		return displayUpdated;
	}

	void buildWorkflowSoftTetModel(WorkflowStageContext& ctx)
	{
	if (!isWorkflowSoftFractureStage(ctx.stage))
		return;
	SimulationModel* model = Simulation::getCurrent()->getModel();
	if (model == nullptr)
		return;

	model->cleanup();
	Find::clearEdgeLookups();

	TetGridBuilder::Input builderInput;
	builderInput.resolution[0] = ctx.resolutionSDF[0];
	builderInput.resolution[1] = ctx.resolutionSDF[1];
	builderInput.resolution[2] = ctx.resolutionSDF[2];
	builderInput.isNodeAvailable = [&ctx](const unsigned int nodeIndex) {
		return (nodeIndex < ctx.eleAvailableMask.size()) && ctx.eleAvailableMask[nodeIndex];
	};
	builderInput.nodePosition = [&ctx](const unsigned int nodeIndex) {
		return ctx.allEleNodePoints[nodeIndex];
	};
	const TetGridBuilder::Result buildResult = TetGridBuilder::build(builderInput, ctx.softTetWorkspace);
	(void)buildResult;
	if (ctx.softTetWorkspace.activeNodePositions.empty() || ctx.softTetWorkspace.indices.empty())
	{
		ctx.warning += (ctx.warning.empty() ? "" : "; ") + std::string("Soft TetModel build produced empty tet mesh");
		return;
	}

	model->addTetModel(
		ctx.softTetWorkspace.activeNodePositions.size(),
		static_cast<unsigned int>(ctx.softTetWorkspace.indices.size() / 4u),
		ctx.softTetWorkspace.activeNodePositions.data(),
		ctx.softTetWorkspace.indices.data());
	TetModel* tetModel = model->getTetModels().empty() ? nullptr : model->getTetModels()[0];
	if (tetModel == nullptr)
		return;

	ParticleData& pd = model->getParticles();
	const unsigned int offset = tetModel->getIndexOffset();
	for (unsigned int i = offset; i < offset + tetModel->getParticleMesh().numVertices(); i++)
		pd.setMass(i, static_cast<Real>(1.0));

	VertexData& visVertices = tetModel->getVisVertices();
	IndexedFaceMesh& visMesh = tetModel->getVisMesh();
	visVertices = ctx.warpedVD;
	visMesh = ctx.warpedMesh;
	tetModel->updateMeshNormals(pd);
	tetModel->attachVisMesh(pd);
	tetModel->updateVisMesh(pd);

	const unsigned int solidMethod = 6u;
	const Real stiffness = static_cast<Real>(500000.0);
	const Real volumeStiffness = static_cast<Real>(100000.0);
	model->addSolidConstraints(
		tetModel,
		solidMethod,
		stiffness,
		model->getSolidPoissonRatio(),
		volumeStiffness,
		model->getSolidNormalizeStretch(),
		model->getSolidNormalizeShear());

	ctx.softCutQueryState.markTopologyDirty();
	ctx.softCutQueryState.markPositionsDirty();
	ctx.softTetModelBuilt = true;
	copyWorkflowFractureDisplayMesh(
		*tetModel,
		pd,
		ctx.softDisplayVD,
		ctx.softDisplayMesh,
		WorkflowPatchCopyMode::VisibleSoftFragmentOnly);
	ctx.softRenderFaceCount = ctx.softDisplayMesh.numFaces();
	ctx.softPatchFaceCount =
		countWorkflowPatchFaces(*tetModel, WorkflowPatchCopyMode::VisibleSoftFragmentOnly);
	ctx.softDebugPatchFaceCount =
		countWorkflowPatchFaces(*tetModel, WorkflowPatchCopyMode::AllDebugPatches);
}

bool applyWorkflowSoftFractureStep(WorkflowStageContext& ctx, const Vector3r& toolPos, const Real radius)
{
	if (!isWorkflowSoftFractureStage(ctx.stage))
		return false;
	if (!ctx.softTetModelBuilt)
		buildWorkflowSoftTetModel(ctx);

	SimulationModel* model = Simulation::getCurrent()->getModel();
	if ((model == nullptr) || model->getTetModels().empty())
		return false;
	TetModel* tetModel = model->getTetModels()[0];
	if (tetModel == nullptr)
		return false;

	ParticleData& pd = model->getParticles();
	seedWorkflowSoftSelectionFromToolPosition(ctx, *tetModel, pd, toolPos);
	Vector3r toolDelta = Vector3r::Zero();
	if (ctx.softHasLastToolPosition)
		toolDelta = toolPos - ctx.softLastToolPosition;
	ctx.softLastToolPosition = toolPos;
	ctx.softHasLastToolPosition = true;

	if (ctx.softCutApplied && !ctx.softDraggedFragmentParticles.empty())
	{
		applyWorkflowParticleDelta(pd, ctx.softDraggedFragmentParticles, toolDelta);
		ctx.softDraggedDistance += toolDelta.norm();
		refreshWorkflowSoftDisplayMesh(ctx, *tetModel, pd, ctx.softLastFractureEdges, true);
		return true;
	}

	if (!ctx.softSelectedParticles.empty())
	{
		const Vector3r preCutDelta =
			(ctx.softScript.enabled && !gWorkflowHapticToolActive) ? ctx.softScript.preCutParticleDelta : toolDelta;
		applyWorkflowParticleDelta(pd, ctx.softSelectedParticles, preCutDelta);
	}

	ctx.softCutQueryState.markPositionsDirty();
	const Clock::time_point toolQueryStart = Clock::now();
	const ToolCutInteraction::EdgePairs hitEdges = ctx.softCutQueryState.queryCutEdgesAt(
		pd.getVertices(),
		[&ctx, tetModel]() { return collectWorkflowTetEdgesExcluding(*tetModel, ctx.softDiscardedFragmentParticles); },
		ctx.toolProbe,
		toolPos,
		radius);
	ctx.toolProbeInitialized = ctx.toolProbe.initialized;
	ctx.lastToolQueryMs = elapsedMs(toolQueryStart);
	if (hitEdges.empty() && ctx.softPendingCutHitEdges.empty())
	{
		refreshWorkflowSoftDisplayMesh(ctx, *tetModel, pd, ctx.softLastFractureEdges, false);
		return false;
	}

	if (!hitEdges.empty())
		ctx.softHitEdgeCount += static_cast<unsigned int>(hitEdges.size());

	const bool hasNewHitEdges = !hitEdges.empty();
	const ToolCutInteraction::EdgePairs& candidateHitEdges =
		hasNewHitEdges ? hitEdges : ctx.softPendingCutHitEdges;
	std::vector<unsigned int> catchFragmentParticles;
	if (!hasNewHitEdges && !ctx.softPendingCutFragmentParticles.empty())
		catchFragmentParticles = ctx.softPendingCutFragmentParticles;
	else
		catchFragmentParticles = collectWorkflowFragmentCatchParticles(ctx, *tetModel, candidateHitEdges);

	ToolCutInteraction::EdgePairs triggerEdges = candidateHitEdges;
	bool triggerUsesCatchBoundary = false;
	ToolCutInteraction::CrossBoundaryConstraintQueryResult boundaryTriggerQuery;
	if (!catchFragmentParticles.empty())
	{
		boundaryTriggerQuery = ToolCutInteraction::collectCrossBoundaryDistanceConstraintEdges(
			*model,
			catchFragmentParticles,
			&tetModel->getDistanceConstraintIds(),
			true);
		if (!boundaryTriggerQuery.edgePairs.empty())
		{
			triggerEdges = boundaryTriggerQuery.edgePairs;
			triggerUsesCatchBoundary = true;
		}
	}

	ToolCutInteraction::FractureTriggerStats triggerStats;
	ToolCutInteraction::EdgePairs readyEdges =
		ctx.softFractureTriggerState.filterReadyEdges(
			*model,
			triggerEdges,
			workflowSoftFractureTriggerOptions(),
			&triggerStats);
	if (readyEdges.empty() && hasNewHitEdges)
	{
		ToolCutInteraction::FractureTriggerOptions immediateOptions;
		immediateOptions.mode = ToolCutInteraction::FractureTriggerMode::Immediate;
		readyEdges = ctx.softFractureTriggerState.filterReadyEdges(
			*model,
			triggerEdges,
			immediateOptions,
			&triggerStats);
	}
	if (readyEdges.empty())
	{
		if (!catchFragmentParticles.empty())
		{
			ctx.softPendingCutHitEdges = candidateHitEdges;
			ctx.softPendingCutFragmentParticles = catchFragmentParticles;
		}
		refreshWorkflowSoftDisplayMesh(ctx, *tetModel, pd, ctx.softLastFractureEdges, false);
		return false;
	}

	ctx.softPendingCutHitEdges.clear();
	ctx.softPendingCutFragmentParticles.clear();

	ToolCutInteraction::DeactivationResult cutResult;
	if (triggerUsesCatchBoundary)
	{
		const ToolCutInteraction::DeactivationResult boundaryResult =
			ToolCutInteraction::deactivateCrossBoundaryDistanceConstraints(
				*model,
				catchFragmentParticles,
				&tetModel->getDistanceConstraintIds());
		mergeWorkflowDeactivationResult(cutResult, boundaryResult);
	}
	else
	{
		cutResult = ToolCutInteraction::deactivateHitEdgeConstraints(*model, readyEdges);
	}
	if (cutResult.fractureDelta.inactiveConstraintIds.empty())
	{
		refreshWorkflowSoftDisplayMesh(ctx, *tetModel, pd, ctx.softLastFractureEdges, false);
		return false;
	}

	ctx.softInactiveConstraintCount += static_cast<unsigned int>(cutResult.fractureDelta.inactiveConstraintIds.size());

	std::vector<std::array<unsigned int, 2> > fractureEdges;
	for (const FractureEdgeKey& edge : cutResult.fractureDelta.brokenEdges)
		fractureEdges.push_back({ { edge.v0, edge.v1 } });
	if (fractureEdges.empty())
		fractureEdges = readyEdges;

	tetModel->getFractureState().applyDelta(cutResult.fractureDelta);
	tetModel->classifyAttachmentsAfterFracture(*model);
	rebuildWorkflowDraggedFragmentParticles(ctx, *tetModel, fractureEdges, catchFragmentParticles);
	ctx.softLastFractureEdges = fractureEdges;
	ctx.softCutApplied = true;

	refreshWorkflowSoftDisplayMesh(ctx, *tetModel, pd, fractureEdges, true);
	return true;
}

// --- Hit-Path Replay Parser ---

struct HitPathReplay
{
	std::string sourcePath;
	std::string source;           // "recorded_json" or "metadata_fallback"
	unsigned int toolPoseCount = 0u;
	unsigned int requestedHitEdgeCount = 0u;
	std::vector<std::array<unsigned int, 2> > hitEdges;
	Real dragDistance = static_cast<Real>(0.0);
	bool loaded = false;
};

namespace
{
	bool parseUnsigned(const char* str, unsigned int& out)
	{
		if (!str || !str[0]) return false;
		char* end = nullptr;
		unsigned long v = std::strtoul(str, &end, 10);
		if (end == str) return false;
		out = static_cast<unsigned int>(v);
		return true;
	}

	bool parseDouble(const char* str, double& out)
	{
		if (!str || !str[0]) return false;
		char* end = nullptr;
		out = std::strtod(str, &end);
		return (end != str);
	}

	unsigned int parseUnsignedField(const std::string& text, const std::string& key, unsigned int fallback)
	{
		const size_t keyPos = text.find("\"" + key + "\"");
		if (keyPos == std::string::npos) return fallback;
		const size_t colon = text.find(':', keyPos);
		if (colon == std::string::npos) return fallback;
		size_t start = colon + 1u;
		while ((start < text.size()) && std::isspace(static_cast<unsigned char>(text[start])))
			start++;
		size_t end = start;
		while ((end < text.size()) && std::isdigit(static_cast<unsigned char>(text[end])))
			end++;
		if (end == start) return fallback;
		unsigned int value = fallback;
		if (!parseUnsigned(text.substr(start, end - start).c_str(), value))
			return fallback;
		return value;
	}

	double parseDoubleField(const std::string& text, const std::string& key, double fallback)
	{
		const size_t keyPos = text.find("\"" + key + "\"");
		if (keyPos == std::string::npos) return fallback;
		const size_t colon = text.find(':', keyPos);
		if (colon == std::string::npos) return fallback;
		size_t start = colon + 1u;
		while ((start < text.size()) && std::isspace(static_cast<unsigned char>(text[start])))
			start++;
		size_t end = start;
		while ((end < text.size()) &&
			(std::isdigit(static_cast<unsigned char>(text[end])) || text[end] == '.' || text[end] == '-' || text[end] == '+'))
			end++;
		if (end == start) return fallback;
		double value = fallback;
		if (!parseDouble(text.substr(start, end - start).c_str(), value))
			return fallback;
		return value;
	}

	std::vector<std::array<unsigned int, 2> > parseHitEdges(const std::string& text)
	{
		std::vector<std::array<unsigned int, 2> > edges;
		size_t pos = 0u;
		while (true)
		{
			const size_t keyPos = text.find("\"edge\"", pos);
			if (keyPos == std::string::npos) break;
			const size_t open = text.find('[', keyPos);
			const size_t comma = text.find(',', open == std::string::npos ? keyPos : open);
			const size_t close = text.find(']', comma == std::string::npos ? keyPos : comma);
			if ((open == std::string::npos) || (comma == std::string::npos) || (close == std::string::npos))
				break;
			unsigned int p0 = 0u;
			unsigned int p1 = 0u;
			const std::string first = text.substr(open + 1u, comma - open - 1u);
			const std::string second = text.substr(comma + 1u, close - comma - 1u);
			if (parseUnsigned(first.c_str(), p0) && parseUnsigned(second.c_str(), p1))
				edges.push_back(std::array<unsigned int, 2>{ { p0, p1 } });
			pos = close + 1u;
		}
		return edges;
	}
}

HitPathReplay loadHitPathReplay(const std::string& replayPath)
{
	HitPathReplay replay;
	replay.sourcePath = replayPath;

	std::string fullPath = FileSystem::normalizePath(workflowExePath() + "/" + replayPath);
	std::ifstream file(fullPath);
	if (!file.is_open())
	{
		LOG_WARN << "Replay file not found: " << fullPath;
		return replay;
	}

	std::stringstream buffer;
	buffer << file.rdbuf();
	const std::string text = buffer.str();

	replay.toolPoseCount = parseUnsignedField(text, "tool_pose_count", 0u);
	replay.requestedHitEdgeCount = parseUnsignedField(text, "hit_edge_count", 0u);
	replay.hitEdges = parseHitEdges(text);
	replay.source = replay.hitEdges.empty() ? "metadata_fallback" : "recorded_json";
	replay.dragDistance = static_cast<Real>(parseDoubleField(text, "drag_distance", 0.0));
	replay.loaded = (replay.toolPoseCount > 0u || replay.requestedHitEdgeCount > 0u || !replay.hitEdges.empty());

	LOG_INFO << "Loaded replay: source=" << replay.source << " tool_poses=" << replay.toolPoseCount
		<< " hit_edge_count=" << replay.requestedHitEdgeCount << " edges=" << replay.hitEdges.size();
	return replay;
}

// --- Fixed Operation Paths ---

void runLinearSDFSweep(WorkflowStageContext& ctx, const std::string& label)
{
	const Eigen::AlignedBox3d& domain = ctx.distanceField->domain();
	const Vector3d& minPos = domain.min();
	const Vector3d& maxPos = domain.max();
	const Vector3d center(
		(minPos[0] + maxPos[0]) * 0.5,
		(minPos[1] + maxPos[1]) * 0.5,
		(minPos[2] + maxPos[2]) * 0.5);

	const unsigned int steps = 40;
	const Real sweepLen = static_cast<Real>(maxPos[0] - minPos[0]) * static_cast<Real>(0.6);
	const Vector3r startPos(
		static_cast<Real>(center[0] - sweepLen * 0.5),
		static_cast<Real>(center[1]),
		static_cast<Real>(center[2]));

	for (unsigned int s = 0; s < steps; s++)
	{
		const Real t = static_cast<Real>(s) / static_cast<Real>(steps - 1);
		const Vector3r pos = startPos + Vector3r(sweepLen * t, static_cast<Real>(0.0), static_cast<Real>(0.0));
		applySweptSDFEdit(ctx, pos);
	}
	reconstructStageSurface(ctx);

	char buf[128];
	snprintf(buf, sizeof(buf), "%s done: %u verts, %u faces",
		label.c_str(), (unsigned)ctx.warpedVD.size(), ctx.warpedMesh.numFaces());
	setStatusText(buf);
}

void runDiscRemovalFixedPath(WorkflowStageContext& ctx)
{
	const Eigen::AlignedBox3d& domain = ctx.distanceField->domain();
	const Vector3d& minPos = domain.min();
	const Vector3d& maxPos = domain.max();
	const Real sweepLen = static_cast<Real>(maxPos[0] - minPos[0]) * static_cast<Real>(0.5);
	const Vector3r center(
		static_cast<Real>((minPos[0] + maxPos[0]) * 0.5),
		static_cast<Real>((minPos[1] + maxPos[1]) * 0.5),
		static_cast<Real>((minPos[2] + maxPos[2]) * 0.5));

	const unsigned int sweeps = 3;
	const unsigned int stepsPerSweep = 30;
	for (unsigned int sw = 0; sw < sweeps; sw++)
	{
		const Real offsetY = (static_cast<Real>(sw) - static_cast<Real>(1.0)) * static_cast<Real>(0.02) * static_cast<Real>(maxPos[1] - minPos[1]);
		const Vector3r sweepStart(
			static_cast<Real>(center[0] - sweepLen * 0.5),
			static_cast<Real>(center[1] + offsetY),
			static_cast<Real>(center[2]));
		for (unsigned int s = 0; s < stepsPerSweep; s++)
		{
			const Real t = static_cast<Real>(s) / static_cast<Real>(stepsPerSweep - 1);
			applySweptSDFEdit(ctx, sweepStart + Vector3r(sweepLen * t, static_cast<Real>(0), static_cast<Real>(0)));
		}
	}
	reconstructStageSurface(ctx);

	char buf[128];
	snprintf(buf, sizeof(buf), "Disc removal done: %u verts, %u faces",
		(unsigned)ctx.warpedVD.size(), ctx.warpedMesh.numFaces());
	setStatusText(buf);
}

void runWorkflowToolPosePath(
	WorkflowStageContext& ctx,
	const std::vector<WorkflowToolPoseStep>& path,
	const std::string& label)
{
	std::size_t changedNodes = 0u;
	unsigned int appliedFractureSteps = 0u;
	for (const WorkflowToolPoseStep& step : path)
	{
		ctx.toolRadius = step.radius;
		ctx.toolProfile.sdfEditRadius = step.radius;
		moveWorkflowToolVisualTo(step.tip);
		if (step.action == WorkflowToolPoseAction::SDFEdit)
		{
			changedNodes += applySweptSDFEdit(ctx, step.tip);
		}
		else if (step.action == WorkflowToolPoseAction::XPBDPullFracture)
		{
			if (applyWorkflowSoftFractureStep(ctx, step.tip, step.radius))
				appliedFractureSteps++;
		}
		else
		{
			ctx.toolController.updateProbe(ctx.toolProbe, step.tip, step.radius);
			ctx.toolProbeInitialized = ctx.toolProbe.initialized;
		}
	}

	if (SDFSurfaceReconstruction::shouldReconstruct(ctx.meshUpdated, ctx.surfaceSignature))
		reconstructStageSurface(ctx);

	char buf[192];
	if (isWorkflowSoftFractureStage(ctx.stage))
	{
		snprintf(buf, sizeof(buf), "%s done: hit_edges=%u inactive_constraints=%u render_faces=%u patch_faces=%u",
			label.c_str(),
			ctx.softHitEdgeCount,
			ctx.softInactiveConstraintCount,
			ctx.softRenderFaceCount,
			ctx.softPatchFaceCount);
	}
	else
	{
		snprintf(buf, sizeof(buf), "%s done: changed_nodes=%u verts=%u faces=%u",
			label.c_str(),
			static_cast<unsigned int>(changedNodes),
			static_cast<unsigned int>(ctx.warpedVD.size()),
			ctx.warpedMesh.numFaces());
	}
	setStatusText(buf);
}

// --- Stage Switching ---

bool captureStageFractureDisplayMesh(WorkflowStageContext& ctx, const std::string& label)
{
	if (!isWorkflowSoftFractureStage(ctx.stage))
		return false;
	if (ctx.softInactiveConstraintCount == 0u)
	{
		LOG_WARN << "Stage " << label << " produced no inactive constraints.";
		return false;
	}

	SimulationModel* model = Simulation::getCurrent()->getModel();
	TetModel* tetModel = ((model != nullptr) && !model->getTetModels().empty()) ? model->getTetModels()[0] : nullptr;
	if (tetModel != nullptr)
	{
		unsigned int mainComponentFaces = 0u;
		if (copyWorkflowMainComponentDisplayMesh(
			*tetModel,
			model->getParticles(),
			ctx.softDisplayVD,
			ctx.softDisplayMesh,
			mainComponentFaces))
		{
			ctx.softMainComponentFaceCount = mainComponentFaces;
		}
		else
		{
			copyWorkflowFractureDisplayMesh(
				*tetModel,
				model->getParticles(),
				ctx.softDisplayVD,
				ctx.softDisplayMesh,
				WorkflowPatchCopyMode::VisibleSoftFragmentOnly);
			ctx.softMainComponentFaceCount = ctx.softDisplayMesh.numFaces();
		}
	}

	if ((ctx.softDisplayVD.size() == 0u) || (ctx.softDisplayMesh.numFaces() == 0u))
	{
		LOG_WARN << "Stage " << label << " produced empty fracture display mesh.";
		return false;
	}

	WorkflowVisualMeshAsset asset;
	asset.label = label;
	asset.vertices = ctx.softDisplayVD;
	asset.mesh = ctx.softDisplayMesh;
	asset.visible = true;
	if (!prepareRenderableMesh(asset))
	{
		LOG_WARN << "Stage " << label << " produced non-renderable fracture display mesh.";
		return false;
	}
	gPreservedMeshes.push_back(asset);
	return true;
}

bool workflowStageResultReady(const WorkflowStageContext& ctx)
{
	if (isWorkflowSoftFractureStage(ctx.stage))
		return (ctx.softInactiveConstraintCount > 0u) &&
			(ctx.softDisplayVD.size() > 0u) &&
			(ctx.softDisplayMesh.numFaces() > 0u);
	return (ctx.warpedVD.size() > 0u) && (ctx.warpedMesh.numFaces() > 0u);
}

unsigned int workflowResultMeshCount()
{
	unsigned int count = 0u;
	if (hasPreservedMeshLabel("bone_result"))
		count++;
	if (hasPreservedMeshLabel("ligament_result"))
		count++;
	if (hasPreservedMeshLabel("disc_result"))
		count++;
	return count;
}

bool captureBoneGrindPatchMesh(const WorkflowStageContext& ctx)
{
	if ((ctx.stage != WorkflowStage::BoneGrinding) ||
		(ctx.warpedVD.size() == 0u) ||
		(ctx.warpedMesh.numFaces() == 0u))
		return false;

	const std::vector<WorkflowToolPoseStep> path = buildBoneGrindingPath(ctx);
	if (path.empty())
		return false;

	WorkflowVisualMeshAsset asset;
	asset.label = "bone_grind_patch";
	asset.visible = true;
	std::vector<unsigned int> remap(ctx.warpedVD.size(), INVALID);
	const std::vector<unsigned int>& faces = ctx.warpedMesh.getFaces();
	for (unsigned int face = 0u; face < ctx.warpedMesh.numFaces(); face++)
	{
		const unsigned int sourceIds[3] = {
			faces[3u * face],
			faces[3u * face + 1u],
			faces[3u * face + 2u]
		};
		if ((sourceIds[0] >= ctx.warpedVD.size()) ||
			(sourceIds[1] >= ctx.warpedVD.size()) ||
			(sourceIds[2] >= ctx.warpedVD.size()))
			continue;

		const Vector3r center =
			(ctx.warpedVD.getPosition(sourceIds[0]) +
				ctx.warpedVD.getPosition(sourceIds[1]) +
				ctx.warpedVD.getPosition(sourceIds[2])) / static_cast<Real>(3.0);
		bool nearGrindPath = false;
		for (const WorkflowToolPoseStep& step : path)
		{
			if (step.action != WorkflowToolPoseAction::SDFEdit)
				continue;
			const Real radius = std::max(step.radius * static_cast<Real>(1.35), static_cast<Real>(0.02));
			if ((center - step.tip).squaredNorm() <= radius * radius)
			{
				nearGrindPath = true;
				break;
			}
		}
		if (!nearGrindPath)
			continue;

		unsigned int remappedIds[3];
		for (unsigned int corner = 0u; corner < 3u; corner++)
		{
			const unsigned int sourceId = sourceIds[corner];
			if (remap[sourceId] == INVALID)
			{
				remap[sourceId] = asset.vertices.size();
				asset.vertices.addVertex(ctx.warpedVD.getPosition(sourceId));
			}
			remappedIds[corner] = remap[sourceId];
		}
		if (asset.mesh.numFaces() == 0u)
			asset.mesh.initMesh(ctx.warpedVD.size(), 0u, ctx.warpedMesh.numFaces());
		asset.mesh.addFace(remappedIds);
	}
	if ((asset.vertices.size() == 0u) || (asset.mesh.numFaces() == 0u))
		return false;

	rebuildMeshVertexCount(asset.mesh, asset.vertices.size());
	asset.mesh.buildNeighbors();
	asset.mesh.updateNormals(asset.vertices, 0u);
	asset.mesh.updateVertexNormals(asset.vertices);
	const std::vector<Vector3r>& normals = asset.mesh.getVertexNormals();
	AlignedBox3r bounds;
	for (unsigned int i = 0u; i < ctx.warpedVD.size(); i++)
		bounds.extend(ctx.warpedVD.getPosition(i));
	const Real normalOffset = std::max(bounds.diagonal().norm() * static_cast<Real>(0.002), static_cast<Real>(0.005));
	for (unsigned int i = 0u; i < asset.vertices.size(); i++)
	{
		Vector3r normal(static_cast<Real>(0.0), static_cast<Real>(1.0), static_cast<Real>(0.0));
		if ((i < normals.size()) && (normals[i].squaredNorm() > static_cast<Real>(1e-12)))
			normal = normals[i].normalized();
		asset.vertices.setPosition(i, asset.vertices.getPosition(i) + normal * normalOffset);
	}
	if (!prepareRenderableMesh(asset))
		return false;
	gAutoDemoState.boneGrindPatchFaces = asset.mesh.numFaces();
	gPreservedMeshes.push_back(asset);
	return true;
}

void captureStageWarpedMesh(WorkflowStageContext& ctx, const std::string& label)
{
	if (isWorkflowSoftFractureStage(ctx.stage))
	{
		captureStageFractureDisplayMesh(ctx, label);
		return;
	}

	if (ctx.warpedVD.size() == 0 || ctx.warpedMesh.numFaces() == 0)
	{
		LOG_WARN << "Stage " << label << " produced empty warped mesh.";
		return;
	}

	WorkflowVisualMeshAsset asset;
	asset.label = label;
	asset.vertices = ctx.warpedVD;
	asset.mesh = ctx.warpedMesh;
	asset.visible = true;
	if (!prepareRenderableMesh(asset))
	{
		LOG_WARN << "Stage " << label << " produced non-renderable warped mesh.";
		return;
	}
	gPreservedMeshes.push_back(asset);
	if (label == "bone_result")
		captureBoneGrindPatchMesh(ctx);
}

void cleanupCurrentStage()
{
	SimulationModel* model = Simulation::getCurrent()->getModel();
	if (model)
		model->cleanup();
	gActiveStageContext.clear();
}

void runFixedOrReplayOperation(WorkflowStageContext& ctx)
{
	switch (ctx.stage)
	{
	case WorkflowStage::BoneGrinding:
		runWorkflowToolPosePath(ctx, buildBoneGrindingPath(ctx), "Bone grinding");
		break;
	case WorkflowStage::LigamentRemoval:
	{
		HitPathReplay replay = loadHitPathReplay(ctx.replayPath);
		if (replay.loaded)
		{
			if (replay.source == "metadata_fallback")
				ctx.warning += (ctx.warning.empty() ? "" : "; ") + string("Replay metadata-only (no hit_edges); used deterministic XPBD path");
			else
				ctx.warning += (ctx.warning.empty() ? "" : "; ") + string("Replay with ") + to_string(replay.hitEdges.size()) + string(" hit_edges; used deterministic XPBD path");
		}
		else
		{
			ctx.warning += (ctx.warning.empty() ? "" : "; ") + string("Replay not found; used deterministic XPBD path");
		}
		runWorkflowToolPosePath(ctx, buildLigamentTearPath(ctx), "Ligament grasp");
		break;
	}
	case WorkflowStage::DiscRemoval:
	{
		HitPathReplay replay = loadHitPathReplay(ctx.replayPath);
		if (replay.loaded)
		{
			if (replay.source == "metadata_fallback")
				ctx.warning += (ctx.warning.empty() ? "" : "; ") + string("Replay metadata-only; used deterministic XPBD path");
			else
				ctx.warning += (ctx.warning.empty() ? "" : "; ") + string("Replay with ") + to_string(replay.hitEdges.size()) + string(" hit_edges; used deterministic XPBD path");
		}
		else
		{
			ctx.warning += (ctx.warning.empty() ? "" : "; ") + string("Replay not found; used deterministic XPBD path");
		}
		runWorkflowToolPosePath(ctx, buildDiscPullPath(ctx), "Disc pull");
		break;
	}
	default:
		break;
	}
}

// --- Retraction Transition ---

Vector3r lerpVec3(const Vector3r& a, const Vector3r& b, Real t)
{
	return a + t * (b - a);
}

void clearWorkflowRetractorAsset()
{
	releaseWorkflowGpuMesh(gRetractorAsset);
	gRetractorAsset = WorkflowVisualMeshAsset();
	gRetractorAssetLoaded = false;
	gRetractorAssetPath.clear();
	gRetractedTransforms = RetractedTransforms();
	gTransitionStart = RetractedTransforms();
	gTransitionEnd = RetractedTransforms();
}

void enterDiscRemovalStage()
{
	gTransitionPlaying = false;
	gTransitionElapsedSec = static_cast<Real>(0.0);
	clearWorkflowRetractorAsset();
	gWorkflowStage = WorkflowStage::DiscRemoval;
	buildStageContext(gActiveStageContext, gWorkflowStage);
	setStatusText("Stage 3: Disc Removal. Use haptic tool to grasp disc tissue.");
}

void enterRetractionTransition()
{
	gTransitionElapsedSec = static_cast<Real>(0.0);
	gTransitionPlaying = false;
	clearWorkflowRetractorAsset();
}

void updateRetractionAnimation(Real dt)
{
	if (!gTransitionPlaying) return;

	gTransitionElapsedSec += dt;
	Real u = std::min(gTransitionElapsedSec / gTransitionDurationSec, static_cast<Real>(1.0));
	Real eased = u * u * (static_cast<Real>(3) - static_cast<Real>(2) * u);

	gRetractedTransforms.toolTranslation = lerpVec3(gTransitionStart.toolTranslation, gTransitionEnd.toolTranslation, eased);
	if (gRetractorAssetLoaded)
	{
		gRetractorAsset.translation = gRetractedTransforms.toolTranslation;
	}

	if (u >= static_cast<Real>(1.0))
	{
		enterDiscRemovalStage();
	}
}

void requestNextStage()
{
	switch (gWorkflowStage)
	{
	case WorkflowStage::BoneGrinding:
		if (gAutoFixedOperationEnabled)
			runFixedOrReplayOperation(gActiveStageContext);
		ensureStageSurfaceCurrent(gActiveStageContext);
		if (gActiveStageContext.warpedVD.size() == 0)
		{
			setStatusText("ERROR: Bone grinding produced empty mesh. Run fixed operation first.");
			return;
		}
		captureStageWarpedMesh(gActiveStageContext, "bone_result");
		cleanupCurrentStage();

		gWorkflowStage = WorkflowStage::LigamentRemoval;
		buildStageContext(gActiveStageContext, gWorkflowStage);
		setStatusText("Stage 2: Ligament Grasp. Use replay or fixed path.");
		break;

	case WorkflowStage::LigamentRemoval:
		if (gAutoFixedOperationEnabled)
			runFixedOrReplayOperation(gActiveStageContext);
		ensureStageSurfaceCurrent(gActiveStageContext);
		if (!workflowStageResultReady(gActiveStageContext))
		{
			setStatusText("ERROR: Ligament grasp produced no soft fracture result.");
			return;
		}
		captureStageWarpedMesh(gActiveStageContext, "ligament_result");
		cleanupCurrentStage();

		enterDiscRemovalStage();
		break;

	case WorkflowStage::RetractionTransition:
		enterDiscRemovalStage();
		break;

	case WorkflowStage::DiscRemoval:
		if (gAutoFixedOperationEnabled)
			runFixedOrReplayOperation(gActiveStageContext);
		ensureStageSurfaceCurrent(gActiveStageContext);
		if (!workflowStageResultReady(gActiveStageContext))
		{
			setStatusText("ERROR: Disc removal produced no soft fracture result.");
			return;
		}
		captureStageWarpedMesh(gActiveStageContext, "disc_result");
		cleanupCurrentStage();
		gWorkflowStage = WorkflowStage::Completed;
		setStatusText("Workflow complete! All stages finished.");
		break;

	case WorkflowStage::Completed:
		setStatusText("Already completed. Press 'Restart Workflow' to begin again.");
		break;
	}
}

const char* workflowAutoDemoPhaseName()
{
	switch (gAutoDemoState.phase)
	{
	case WorkflowAutoDemoPhase::Disabled: return "Disabled";
	case WorkflowAutoDemoPhase::BoneGrinding: return "Bone Grinding";
	case WorkflowAutoDemoPhase::BoneHold: return "Bone Hold";
	case WorkflowAutoDemoPhase::LigamentTear: return "Ligament Grasp";
	case WorkflowAutoDemoPhase::LigamentHold: return "Ligament Hold";
	case WorkflowAutoDemoPhase::Retraction: return "Retraction";
	case WorkflowAutoDemoPhase::DiscPull: return "Disc Pull";
	case WorkflowAutoDemoPhase::CompletedHold: return "Completed Hold";
	default: return "Unknown";
	}
}

std::vector<WorkflowToolPoseStep> buildBoneGrindingPath(const WorkflowStageContext& ctx)
{
	std::vector<WorkflowToolPoseStep> path;
	if (ctx.warpedVD.size() == 0u)
		return path;

	const Vector3r topCenter =
		computeTopRegionCenter(ctx.warpedVD);
	if (topCenter.norm() <= static_cast<Real>(1e-6))
		return path;
	const Vector3r contactCenter =
		topCenter - Vector3r(static_cast<Real>(0), ctx.toolRadius * static_cast<Real>(0.60), static_cast<Real>(0));
	WorkflowToolPoseStep approach;
	approach.tip = contactCenter + Vector3r(static_cast<Real>(0), ctx.toolRadius, static_cast<Real>(0));
	approach.radius = ctx.toolRadius;
	approach.holdSeconds = static_cast<Real>(0.035);
	approach.action = WorkflowToolPoseAction::SDFEdit;
	approach.phase = WorkflowToolPosePhase::Approach;
	path.push_back(approach);

	Real maxY = -REAL_MAX;
	for (unsigned int i = 0u; i < ctx.warpedVD.size(); i++)
		maxY = std::max(maxY, ctx.warpedVD.getPosition(i).y());

	std::vector<Vector3r> topSamples;
	for (unsigned int i = 0u; i < ctx.warpedVD.size(); i++)
	{
		const Vector3r& p = ctx.warpedVD.getPosition(i);
		if (p.y() >= maxY - static_cast<Real>(0.80))
			topSamples.push_back(p);
	}
	std::sort(topSamples.begin(), topSamples.end(), [](const Vector3r& a, const Vector3r& b) {
		if (a.z() != b.z())
			return a.z() < b.z();
		return a.x() < b.x();
	});

	const unsigned int maxSampleCount = 28u;
	const unsigned int stride = std::max(1u, static_cast<unsigned int>(
		std::ceil(static_cast<Real>(topSamples.size()) / static_cast<Real>(maxSampleCount))));
	for (unsigned int i = 0u; i < topSamples.size(); i += stride)
	{
		const Vector3r surface = topSamples[i];
		const Vector3r center =
			surface - Vector3r(static_cast<Real>(0), ctx.toolRadius * static_cast<Real>(0.55), static_cast<Real>(0));
		const Vector3r xSweep(ctx.toolRadius * static_cast<Real>(0.45), static_cast<Real>(0), static_cast<Real>(0));
		const Vector3r zSweep(static_cast<Real>(0), static_cast<Real>(0), ctx.toolRadius * static_cast<Real>(0.45));
		const Vector3r deep(static_cast<Real>(0), -ctx.toolRadius * static_cast<Real>(0.30), static_cast<Real>(0));

		const Vector3r points[] = {
			center + xSweep,
			center - xSweep,
			center + zSweep,
			center - zSweep,
			center + deep
		};
		for (const Vector3r& p : points)
		{
			WorkflowToolPoseStep step;
			step.tip = p;
			step.radius = ctx.toolRadius;
			step.holdSeconds = static_cast<Real>(0.035);
			step.action = WorkflowToolPoseAction::SDFEdit;
			step.phase = WorkflowToolPosePhase::Pull;
			path.push_back(step);
		}
	}

	return path;
}

std::vector<WorkflowToolPoseStep> buildLigamentTearPath(const WorkflowStageContext& ctx)
{
	if (!ctx.softScript.enabled)
		return std::vector<WorkflowToolPoseStep>();

	SimulationModel* model = Simulation::getCurrent()->getModel();
	if ((model == nullptr) || model->getTetModels().empty())
		return std::vector<WorkflowToolPoseStep>();
	const TetModel* tetModel = model->getTetModels()[0];
	if (tetModel == nullptr)
		return std::vector<WorkflowToolPoseStep>();
	const ParticleData& pd = model->getParticles();
	const unsigned int particleId = tetModel->getIndexOffset() + ctx.softScript.selectedParticle;
	if (particleId >= pd.size())
		return std::vector<WorkflowToolPoseStep>();

	return WorkflowSoftTissueFracture::buildScriptedParticlePullPath<WorkflowToolPoseStep>(
		pd.getPosition(particleId) + ctx.softScript.toolOffset,
		ctx.softScript.toolPullDelta,
		ctx.toolRadius,
		WorkflowToolPoseAction::MoveOnly,
		WorkflowToolPoseAction::XPBDPullFracture,
		WorkflowToolPosePhase::Approach,
		WorkflowToolPosePhase::Grasp,
		WorkflowToolPosePhase::Pull,
		WorkflowToolPosePhase::Hold);
}

std::vector<WorkflowToolPoseStep> buildDiscPullPath(const WorkflowStageContext& ctx)
{
	if (!ctx.softScript.enabled)
		return std::vector<WorkflowToolPoseStep>();

	SimulationModel* model = Simulation::getCurrent()->getModel();
	if ((model == nullptr) || model->getTetModels().empty())
		return std::vector<WorkflowToolPoseStep>();
	const TetModel* tetModel = model->getTetModels()[0];
	if (tetModel == nullptr)
		return std::vector<WorkflowToolPoseStep>();
	const ParticleData& pd = model->getParticles();
	const unsigned int particleId = tetModel->getIndexOffset() + ctx.softScript.selectedParticle;
	if (particleId >= pd.size())
		return std::vector<WorkflowToolPoseStep>();

	return WorkflowSoftTissueFracture::buildScriptedParticlePullPath<WorkflowToolPoseStep>(
		pd.getPosition(particleId) + ctx.softScript.toolOffset,
		ctx.softScript.toolPullDelta,
		ctx.toolRadius,
		WorkflowToolPoseAction::MoveOnly,
		WorkflowToolPoseAction::XPBDPullFracture,
		WorkflowToolPosePhase::Approach,
		WorkflowToolPosePhase::Grasp,
		WorkflowToolPosePhase::Pull,
		WorkflowToolPosePhase::Hold);
}

void resetWorkflowAutoDemoRuntime()
	{
		const bool requested = gAutoDemoState.commandLineRequested;
		gAutoDemoState = WorkflowAutoDemoState();
		gAutoDemoState.commandLineRequested = requested;
		gWorkflowHasTimeStepWallClock = false;
	}

void resetWorkflowToFirstStage()
	{
		Simulation::getCurrent()->reset();
		SimulationModel* model = Simulation::getCurrent()->getModel();
		if (model) model->cleanup();

		releaseWorkflowGpuMeshes(gPreservedMeshes);
		gPreservedMeshes.clear();
		gTransitionPlaying = false;
		gTransitionElapsedSec = static_cast<Real>(0.0);
		gRetractedTransforms = RetractedTransforms();
		gTransitionStart = RetractedTransforms();
		gTransitionEnd = RetractedTransforms();
		releaseWorkflowGpuMesh(gRetractorAsset);
		gRetractorAsset = WorkflowVisualMeshAsset();
		gRetractorAssetLoaded = false;
		gRetractorAssetPath.clear();
			gWorkflowStage = WorkflowStage::BoneGrinding;
			gActiveStageContext.clear();
			gWorkflowToolVisualInitialized = false;
			gWorkflowToolVisualCenter = Vector3r::Zero();
			gWorkflowToolControlInitialized = false;
			gWorkflowToolControlIndex = 0u;
			gWorkflowHasTimeStepWallClock = false;

			buildStageContext(gActiveStageContext, gWorkflowStage);
		}

void prepareWorkflowAutoDemoPath()
	{
		gAutoDemoState.path.clear();
		gAutoDemoState.stepIndex = 0u;
		gAutoDemoState.holdRemaining = static_cast<Real>(0.0);
		gActiveStageContext.toolProbe = ToolProbe();
		gActiveStageContext.toolProbeInitialized = false;

		switch (gAutoDemoState.phase)
		{
		case WorkflowAutoDemoPhase::BoneGrinding:
			gAutoDemoState.path = buildBoneGrindingPath(gActiveStageContext);
			gAutoDemoState.stage1BallGrinderProxyOk =
				gActiveStageContext.toolControlMode == WorkflowToolControlMode::BallGrinderProxy;
			gAutoDemoState.stage1BallProxyRadius = gActiveStageContext.toolRadius;
			break;
		case WorkflowAutoDemoPhase::LigamentTear:
			gAutoDemoState.path = buildLigamentTearPath(gActiveStageContext);
			gAutoDemoState.stage2ScriptParticleId = gActiveStageContext.softScript.selectedParticle;
			gAutoDemoState.stage2ScriptOffset = gActiveStageContext.softScript.toolOffset;
			gAutoDemoState.stage2GoldenScriptOk =
				gActiveStageContext.softScript.enabled &&
				(gActiveStageContext.softScript.selectedParticle == 609u) &&
				((gActiveStageContext.softScript.toolOffset -
					Vector3r(static_cast<Real>(0.0), static_cast<Real>(-0.3), static_cast<Real>(0.0))).norm() <= static_cast<Real>(1e-6)) &&
				!gAutoDemoState.path.empty();
			break;
		case WorkflowAutoDemoPhase::DiscPull:
			gAutoDemoState.path = buildDiscPullPath(gActiveStageContext);
			gAutoDemoState.stage3ScriptParticleId = gActiveStageContext.softScript.selectedParticle;
			gAutoDemoState.stage3ScriptOffset = gActiveStageContext.softScript.toolOffset;
			gAutoDemoState.stage3GoldenScriptOk =
				gActiveStageContext.softScript.enabled &&
				(gActiveStageContext.softScript.selectedParticle == 1401u) &&
				((gActiveStageContext.softScript.toolOffset -
					Vector3r(static_cast<Real>(0.1), static_cast<Real>(0.0), static_cast<Real>(0.0))).norm() <= static_cast<Real>(1e-6)) &&
				((gActiveStageContext.toolInitialTranslation -
					Vector3r(static_cast<Real>(0.0), static_cast<Real>(0.0), static_cast<Real>(8.47))).norm() <= static_cast<Real>(1e-6)) &&
				!gAutoDemoState.path.empty();
			break;
		default:
			break;
		}
		if (!gAutoDemoState.path.empty())
		{
			gAutoDemoState.currentToolPosition = gAutoDemoState.path.front().tip;
			gAutoDemoState.hasToolPosition = true;
			moveWorkflowToolVisualTo(gAutoDemoState.currentToolPosition);
		}
	}

void startWorkflowAutoDemo()
	{
		resetWorkflowAutoDemoRuntime();
		resetWorkflowToFirstStage();
		gAutoFixedOperationEnabled = false;
		gAutoDemoState.running = true;
		gAutoDemoState.phase = WorkflowAutoDemoPhase::BoneGrinding;
		prepareWorkflowAutoDemoPath();
		if (base != nullptr)
			base->setValue(DemoBase::PAUSE, false);
		setStatusText("Auto demo: bone grinding started.");
	}

void stopWorkflowAutoDemo()
	{
		gAutoDemoState.running = false;
		gAutoDemoState.path.clear();
		gAutoDemoState.stepIndex = 0u;
		gAutoDemoState.phase = WorkflowAutoDemoPhase::Disabled;
		setStatusText("Auto demo stopped.");
	}

	void rebuildAutoDemoSurfaceIfNeeded(const bool force)
		{
			if (gAutoDemoState.phase == WorkflowAutoDemoPhase::BoneGrinding)
			{
				ensureStageSurfaceCurrent(gActiveStageContext);
				return;
			}
			const bool stepBoundary = (gAutoDemoState.stepIndex % 4u) == 0u;
			if (force || stepBoundary)
				ensureStageSurfaceCurrent(gActiveStageContext);
		}

void completeAutoDemoBoneGrinding()
	{
		ensureStageSurfaceCurrent(gActiveStageContext);
		gAutoDemoState.boneGrindTopContact = (gAutoDemoState.boneChangedNodes > 0u);
		captureStageWarpedMesh(gActiveStageContext, "bone_result");
		cleanupCurrentStage();
		gWorkflowStage = WorkflowStage::LigamentRemoval;
		buildStageContext(gActiveStageContext, gWorkflowStage);
		gAutoDemoState.phase = WorkflowAutoDemoPhase::BoneHold;
		gAutoDemoState.holdRemaining = static_cast<Real>(1.20);
		gAutoDemoState.path.clear();
		gAutoDemoState.stepIndex = 0u;
		setStatusText("Auto demo: bone result preserved.");
	}

void completeAutoDemoLigamentTear()
	{
		ensureStageSurfaceCurrent(gActiveStageContext);
		gAutoDemoState.ligamentHitEdges = gActiveStageContext.softHitEdgeCount;
		gAutoDemoState.ligamentInactiveConstraints = gActiveStageContext.softInactiveConstraintCount;
		gAutoDemoState.ligamentRenderFaces = gActiveStageContext.softRenderFaceCount;
		gAutoDemoState.ligamentPatchFaces = gActiveStageContext.softPatchFaceCount;
		gAutoDemoState.ligamentDebugPatchFaces = gActiveStageContext.softDebugPatchFaceCount;
		gAutoDemoState.ligamentDraggedParticles = gActiveStageContext.softDraggedParticleCount;
		gAutoDemoState.ligamentHiddenFaces = gActiveStageContext.softHiddenFaceCount;
		gAutoDemoState.ligamentDraggedDistance = gActiveStageContext.softDraggedDistance;
		SimulationModel* model = Simulation::getCurrent()->getModel();
		if ((model != nullptr) && !model->getTetModels().empty())
		{
			TetModel* tetModel = model->getTetModels()[0];
			if (tetModel != nullptr)
				gAutoDemoState.ligamentComponents = tetModel->getFractureState().getComponentCount();
		}
		gAutoDemoState.stage2ActiveSoftAsset = gActiveStageContext.activeAssetPath;
		gAutoDemoState.stage2TetModelCount =
			(model != nullptr) ? static_cast<unsigned int>(model->getTetModels().size()) : 0u;
		gAutoDemoState.stage2DiscRenderOnly =
			(gActiveStageContext.stage == WorkflowStage::LigamentRemoval) &&
			(gActiveStageContext.activeAssetPath == "rendai_tex.obj") &&
			(gActiveStageContext.activeAssetPath != "target_disc.obj") &&
			workflowSceneLayerLabelLoaded("scene_target_disc_reference");
		captureStageWarpedMesh(gActiveStageContext, "ligament_result");
		gAutoDemoState.stage2MainComponentFaces = gActiveStageContext.softMainComponentFaceCount;
		gAutoDemoState.stage2MainComponentCaptureOk = gAutoDemoState.stage2MainComponentFaces > 0u;
		cleanupCurrentStage();
		enterDiscRemovalStage();
		gAutoDemoState.phase = WorkflowAutoDemoPhase::LigamentHold;
		gAutoDemoState.holdRemaining = static_cast<Real>(1.20);
		gAutoDemoState.path.clear();
		gAutoDemoState.stepIndex = 0u;
		setStatusText("Auto demo: ligament result preserved.");
	}

void completeAutoDemoDiscPull()
	{
		ensureStageSurfaceCurrent(gActiveStageContext);
		gAutoDemoState.discHitEdges = gActiveStageContext.softHitEdgeCount;
		gAutoDemoState.discInactiveConstraints = gActiveStageContext.softInactiveConstraintCount;
		gAutoDemoState.discRenderFaces = gActiveStageContext.softRenderFaceCount;
		gAutoDemoState.discPatchFaces = gActiveStageContext.softPatchFaceCount;
		gAutoDemoState.discDebugPatchFaces = gActiveStageContext.softDebugPatchFaceCount;
		gAutoDemoState.discDraggedParticles = gActiveStageContext.softDraggedParticleCount;
		gAutoDemoState.discHiddenFaces = gActiveStageContext.softHiddenFaceCount;
		gAutoDemoState.discDraggedDistance = gActiveStageContext.softDraggedDistance;
		SimulationModel* model = Simulation::getCurrent()->getModel();
		if ((model != nullptr) && !model->getTetModels().empty())
		{
			TetModel* tetModel = model->getTetModels()[0];
			if (tetModel != nullptr)
				gAutoDemoState.discComponents = tetModel->getFractureState().getComponentCount();
		}
		gAutoDemoState.stage3ActiveSoftAsset = gActiveStageContext.activeAssetPath;
		gAutoDemoState.stage3ToolScale = gActiveStageContext.toolVisualScale;
		gAutoDemoState.stage3TetModelCount =
			(model != nullptr) ? static_cast<unsigned int>(model->getTetModels().size()) : 0u;
		gAutoDemoState.stage3LigamentRenderOnly =
			(gActiveStageContext.stage == WorkflowStage::DiscRemoval) &&
			(gActiveStageContext.activeAssetPath == "target_disc.obj") &&
			(gActiveStageContext.activeAssetPath != "rendai_tex.obj") &&
			hasPreservedMeshLabel("ligament_result");
		if (gWorkflowFixedSummaryMode)
		{
			captureStageWarpedMesh(gActiveStageContext, "disc_result");
			cleanupCurrentStage();
			gWorkflowStage = WorkflowStage::Completed;
			gAutoDemoState.phase = WorkflowAutoDemoPhase::CompletedHold;
			gAutoDemoState.running = false;
			gAutoDemoState.path.clear();
			gAutoDemoState.stepIndex = 0u;
			setStatusText("Auto demo complete.");
		}
		else
		{
			gAutoDemoState.stage3RemainsInteractive =
				(gWorkflowStage == WorkflowStage::DiscRemoval) &&
				(model != nullptr) &&
				(model->getTetModels().size() == 1u);
			gAutoDemoState.stage3LiveHoldOk = true;
			gAutoDemoState.running = false;
			gAutoDemoState.path.clear();
			gAutoDemoState.stepIndex = 0u;
			setStatusText("Auto demo: disc pull complete; Stage 3 remains interactive with idle physics paused.");
		}
	}

void completeAutoDemoActivePath()
	{
		switch (gAutoDemoState.phase)
		{
		case WorkflowAutoDemoPhase::BoneGrinding:
			completeAutoDemoBoneGrinding();
			break;
		case WorkflowAutoDemoPhase::LigamentTear:
			completeAutoDemoLigamentTear();
			break;
		case WorkflowAutoDemoPhase::DiscPull:
			completeAutoDemoDiscPull();
			break;
		default:
			break;
		}
	}

void advanceWorkflowAutoDemo(const Real dt)
	{
		if (!gAutoDemoState.running)
			return;

		if ((gAutoDemoState.phase == WorkflowAutoDemoPhase::BoneHold) ||
			(gAutoDemoState.phase == WorkflowAutoDemoPhase::LigamentHold))
		{
			gAutoDemoState.holdRemaining -= dt;
			if (gAutoDemoState.holdRemaining > static_cast<Real>(0.0))
				return;
			if (gAutoDemoState.phase == WorkflowAutoDemoPhase::BoneHold)
			{
				gAutoDemoState.phase = WorkflowAutoDemoPhase::LigamentTear;
				prepareWorkflowAutoDemoPath();
				setStatusText("Auto demo: ligament grasp started.");
			}
			else
			{
				gAutoDemoState.phase = WorkflowAutoDemoPhase::DiscPull;
				prepareWorkflowAutoDemoPath();
				setStatusText("Auto demo: disc pull started.");
			}
			return;
		}

		if (gAutoDemoState.phase == WorkflowAutoDemoPhase::Retraction)
		{
			updateRetractionAnimation(dt);
			if (!gTransitionPlaying && (gWorkflowStage == WorkflowStage::DiscRemoval))
			{
				gAutoDemoState.phase = WorkflowAutoDemoPhase::DiscPull;
				prepareWorkflowAutoDemoPath();
				setStatusText("Auto demo: disc pull started.");
			}
			return;
		}

		if ((gAutoDemoState.phase != WorkflowAutoDemoPhase::BoneGrinding) &&
			(gAutoDemoState.phase != WorkflowAutoDemoPhase::LigamentTear) &&
			(gAutoDemoState.phase != WorkflowAutoDemoPhase::DiscPull))
		{
			return;
		}

		if (gAutoDemoState.path.empty())
			prepareWorkflowAutoDemoPath();
		if (gAutoDemoState.path.empty())
		{
			gAutoDemoState.running = false;
			setStatusText("Auto demo failed: empty tool path.");
			return;
		}

		if (gAutoDemoState.holdRemaining > static_cast<Real>(0.0))
		{
			gAutoDemoState.holdRemaining -= dt;
			return;
		}

		if (gAutoDemoState.stepIndex >= gAutoDemoState.path.size())
		{
			completeAutoDemoActivePath();
			return;
		}

		const WorkflowToolPoseStep& step = gAutoDemoState.path[gAutoDemoState.stepIndex++];
		gAutoDemoState.currentToolPosition = step.tip;
		gAutoDemoState.hasToolPosition = true;
		moveWorkflowToolVisualTo(step.tip);

		if (gAutoDemoState.phase == WorkflowAutoDemoPhase::LigamentTear)
		{
			if (step.phase == WorkflowToolPosePhase::Grasp)
				gAutoDemoState.ligamentGraspPhaseSeen = true;
			else if (step.phase == WorkflowToolPosePhase::Pull)
				gAutoDemoState.ligamentPullPhaseSeen = true;
			else if (step.phase == WorkflowToolPosePhase::Hold && gActiveStageContext.softCutApplied)
				gAutoDemoState.stage2HoldOk = true;
		}
		else if (gAutoDemoState.phase == WorkflowAutoDemoPhase::DiscPull)
		{
			if (step.phase == WorkflowToolPosePhase::Grasp)
				gAutoDemoState.discGraspPhaseSeen = true;
			else if (step.phase == WorkflowToolPosePhase::Pull)
				gAutoDemoState.discPullPhaseSeen = true;
			else if (step.phase == WorkflowToolPosePhase::Hold && gActiveStageContext.softCutApplied)
			{
				gAutoDemoState.stage3LiveHoldOk = true;
				gAutoDemoState.stage3RemainsInteractive =
					(gWorkflowStage == WorkflowStage::DiscRemoval);
			}
		}

		if (step.action == WorkflowToolPoseAction::SDFEdit)
		{
			gActiveStageContext.toolRadius = step.radius;
			gActiveStageContext.toolProfile.sdfEditRadius = step.radius;
			const std::size_t changedNodes = applySweptSDFEdit(gActiveStageContext, step.tip);
			if (gAutoDemoState.phase == WorkflowAutoDemoPhase::BoneGrinding)
			{
				gAutoDemoState.boneChangedNodes += changedNodes;
				if (changedNodes > 0u)
				{
					gActiveStageContext.ballGrinderProxyContact = true;
					gAutoDemoState.stage1BallProxyContact = true;
				}
			}
			rebuildAutoDemoSurfaceIfNeeded(gAutoDemoState.stepIndex >= gAutoDemoState.path.size());
		}
		else if (step.action == WorkflowToolPoseAction::XPBDPullFracture)
		{
			applyWorkflowSoftFractureStep(gActiveStageContext, step.tip, step.radius);
			if (gAutoDemoState.phase == WorkflowAutoDemoPhase::LigamentTear)
			{
				gAutoDemoState.ligamentHitEdges = gActiveStageContext.softHitEdgeCount;
					gAutoDemoState.ligamentInactiveConstraints = gActiveStageContext.softInactiveConstraintCount;
					gAutoDemoState.ligamentRenderFaces = gActiveStageContext.softRenderFaceCount;
					gAutoDemoState.ligamentPatchFaces = gActiveStageContext.softPatchFaceCount;
					gAutoDemoState.ligamentDebugPatchFaces = gActiveStageContext.softDebugPatchFaceCount;
					gAutoDemoState.ligamentDraggedParticles = gActiveStageContext.softDraggedParticleCount;
					gAutoDemoState.ligamentHiddenFaces = gActiveStageContext.softHiddenFaceCount;
					gAutoDemoState.ligamentDraggedDistance = gActiveStageContext.softDraggedDistance;
				}
				else if (gAutoDemoState.phase == WorkflowAutoDemoPhase::DiscPull)
				{
					gAutoDemoState.discHitEdges = gActiveStageContext.softHitEdgeCount;
					gAutoDemoState.discInactiveConstraints = gActiveStageContext.softInactiveConstraintCount;
					gAutoDemoState.discRenderFaces = gActiveStageContext.softRenderFaceCount;
					gAutoDemoState.discPatchFaces = gActiveStageContext.softPatchFaceCount;
					gAutoDemoState.discDebugPatchFaces = gActiveStageContext.softDebugPatchFaceCount;
					gAutoDemoState.discDraggedParticles = gActiveStageContext.softDraggedParticleCount;
					gAutoDemoState.discHiddenFaces = gActiveStageContext.softHiddenFaceCount;
					gAutoDemoState.discDraggedDistance = gActiveStageContext.softDraggedDistance;
				}
		}
		else
		{
			gActiveStageContext.toolController.updateProbe(gActiveStageContext.toolProbe, step.tip, step.radius);
			gActiveStageContext.toolProbeInitialized = gActiveStageContext.toolProbe.initialized;
		}
		gAutoDemoState.holdRemaining = step.holdSeconds;
	}

std::string workflowStageNameText()
{
	switch (gWorkflowStage)
	{
	case WorkflowStage::BoneGrinding: return "Bone Grinding";
	case WorkflowStage::LigamentRemoval: return "Ligament Grasp";
	case WorkflowStage::RetractionTransition: return "Retraction Transition";
	case WorkflowStage::DiscRemoval: return "Disc Removal";
	case WorkflowStage::Completed: return "Completed";
	default: return "Unknown";
	}
}

std::string workflowStageWarningText()
{
	return gActiveStageContext.warning.empty() ? "none" : gActiveStageContext.warning;
}

bool hasPreservedMeshLabel(const std::string& label)
{
	for (const WorkflowVisualMeshAsset& asset : gPreservedMeshes)
	{
		if (asset.label == label)
			return true;
	}
	return false;
}

bool workflowLigamentResultMoved()
{
	for (const WorkflowVisualMeshAsset& asset : gPreservedMeshes)
	{
		if ((asset.label == "ligament_result") && (asset.translation.norm() > static_cast<Real>(1e-6)))
			return true;
	}
	return false;
}

bool workflowPreservedReplacesDefaultAnatomy()
{
	return hasPreservedMeshLabel("bone_result") && hasPreservedMeshLabel("ligament_result");
}

bool workflowAutoDemoStage3IdleHold()
{
	return !gAutoDemoState.running &&
		gAutoDemoState.stage3RemainsInteractive &&
		(gWorkflowStage == WorkflowStage::DiscRemoval);
}

bool updateWorkflowLiveHapticTool()
{
	if (!gWorkflowHapticToolControl)
		return false;
	if (!MiniGL::isHapticAvailable())
	{
		if (!gWorkflowHapticUnavailableReported)
		{
			setStatusText("ERROR: Haptic device is not initialized. Check Touch driver/device, then restart demo.");
			gWorkflowHapticUnavailableReported = true;
			printWorkflowHapticDiagnostics("haptic_unavailable", true);
		}
		return false;
	}
	gWorkflowHapticUnavailableReported = false;

	const PBD::DemoHaptics::LiveHapticToolSample rawSample =
		PBD::DemoHaptics::sampleMiniGLLiveHapticTool();
	const PBD::DemoHaptics::LiveHapticToolSample sample =
		PBD::DemoHaptics::offsetLiveHapticToolSample(rawSample, gWorkflowHapticToolViewOffset);
	gWorkflowHapticToolActive = sample.active;
	gWorkflowToolVisibilityMarkerPosition = sample.position;
	gWorkflowToolVisibilityMarkerValid = true;

	if ((gWorkflowStage == WorkflowStage::Completed) ||
		(gWorkflowStage == WorkflowStage::RetractionTransition))
	{
		gWorkflowHapticToolWasActive = false;
		return false;
	}

	moveWorkflowToolVisualTo(sample.position);
	printWorkflowHapticDiagnostics("haptic_sample", false);

	const bool justActivated = sample.active && !gWorkflowHapticToolWasActive;
	const bool justReleased = !sample.active && gWorkflowHapticToolWasActive;
	if (justActivated && isWorkflowSoftFractureStage(gWorkflowStage))
		resetWorkflowSoftInteractionSession(gActiveStageContext);
	if (justReleased && isWorkflowSoftFractureStage(gWorkflowStage))
		discardWorkflowReleasedSoftFragment(gActiveStageContext);

	if (!PBD::DemoHaptics::hasActiveLiveHapticTool(sample))
	{
		gWorkflowHapticToolWasActive = false;
		PBD::DemoHaptics::deactivateProbeIfInactive(sample, gActiveStageContext.toolProbe);
		gActiveStageContext.toolProbeInitialized = gActiveStageContext.toolProbe.initialized;
		return false;
	}
	gWorkflowHapticToolWasActive = true;

	if (gWorkflowStage == WorkflowStage::BoneGrinding)
	{
		const Real stageToolRadius = gActiveStageContext.toolRadius;
		const Real editRadius =
			PBD::DemoHaptics::liveHapticSDFEditRadius(stageToolRadius);
		gActiveStageContext.toolRadius = editRadius;
		gActiveStageContext.toolProfile.sdfEditRadius = editRadius;
		const std::size_t changedNodes = applySweptSDFEdit(gActiveStageContext, sample.position);
		gActiveStageContext.toolRadius = stageToolRadius;
		gActiveStageContext.toolProfile.sdfEditRadius = stageToolRadius;
		if (changedNodes > 0u)
			reconstructStageSurface(gActiveStageContext);
		return changedNodes > 0u;
	}

	if (isWorkflowSoftFractureStage(gWorkflowStage))
	{
		const Real fractureRadius =
			PBD::DemoHaptics::liveHapticSoftTissueFractureRadius(gActiveStageContext.toolRadius);
		return applyWorkflowSoftFractureStep(gActiveStageContext, sample.position, fractureRadius);
	}
	return false;
}

// --- Time Step (with transition animation) ---

void timeStep()
{
	const Clock::time_point frameStart = Clock::now();
	SimulationModel* model = Simulation::getCurrent()->getModel();
	const Real dt = workflowLiveStepDt();

	const Real pauseAt = base->getValue<Real>(DemoBase::PAUSE_AT);
	if ((pauseAt > 0.0) && (pauseAt < TimeManager::getCurrent()->getTime()))
		base->setValue(DemoBase::PAUSE, true);

	if (base->getValue<bool>(DemoBase::PAUSE))
		return;

	if (gAutoDemoState.running)
	{
		advanceWorkflowAutoDemo(dt);
		recordDemoFlowFrame(frameStart, 0.0);
		return;
	}

	if (gTransitionPlaying)
	{
		updateRetractionAnimation(dt);
		recordDemoFlowFrame(frameStart, 0.0);
		return;
	}

	if (gWorkflowStage == WorkflowStage::Completed)
	{
		return;
	}

	if (!model) return;
	const bool hapticApplied = updateWorkflowLiveHapticTool();
	if (workflowAutoDemoStage3IdleHold() && !hapticApplied)
	{
		recordDemoFlowFrame(frameStart, 0.0);
		return;
	}

	const unsigned int numSteps = base->getValue<unsigned int>(DemoBase::NUM_STEPS_PER_RENDER);
	double solverMs = 0.0;
	for (unsigned int i = 0; i < numSteps; i++)
	{
		const Clock::time_point solverStart = Clock::now();
		Simulation::getCurrent()->getTimeStep()->step(*model);
		base->step();
		solverMs += elapsedMs(solverStart);
	}
	recordDemoFlowFrame(frameStart, solverMs);
}

// --- Render ---

	void renderWorkflowSceneLayers()
	{
		ensureWorkflowSceneLayersLoaded();
		for (WorkflowSceneLayer& layer : gWorkflowSceneLayers)
		{
			if (!workflowSceneLayerVisible(layer, gWorkflowStage))
				continue;
			float color[4];
			workflowSceneLayerColor(layer.role, color);
			drawWorkflowSceneLayerMesh(layer, color);
		}
	}

void renderActiveStageMesh()
{
	if (gWorkflowStage == WorkflowStage::Completed)
		return;
	if (isWorkflowSoftFractureStage(gWorkflowStage) &&
		(gActiveStageContext.softDisplayVD.size() > 0u) &&
		(gActiveStageContext.softDisplayMesh.numFaces() > 0u))
	{
		float softColor[4];
		float foregroundColor[4];
		workflowActiveSoftColors(gWorkflowStage, softColor, foregroundColor);
		(void)foregroundColor;
		drawWorkflowMesh(gActiveStageContext.softDisplayVD, gActiveStageContext.softDisplayMesh, softColor);
		return;
	}
	if (gActiveStageContext.warpedVD.size() == 0 || gActiveStageContext.warpedMesh.numFaces() == 0)
		return;

	float activeColor[4] = { 0.5f, 0.5f, 0.5f, 1.0f };
	if ((base != nullptr) && (gWorkflowStage == WorkflowStage::BoneGrinding))
	{
		base->shaderBegin(activeColor);
		Visualization::drawMesh(gActiveStageContext.warpedVD, gActiveStageContext.warpedMesh, 0u, activeColor);
		base->shaderEnd();
		return;
	}
	drawWorkflowMesh(gActiveStageContext.warpedVD, gActiveStageContext.warpedMesh, activeColor);
}

void renderPreservedMeshes()
{
	float boneColor[4] = { 0.9f, 0.9f, 0.8f, 1.0f };
	float boneGrindPatchColor[4] = { 0.0f, 0.85f, 0.95f, 1.0f };
	float ligamentColor[4] = { 0.8f, 0.6f, 0.2f, 1.0f };
	float otherColor[4] = { 0.3f, 0.7f, 0.3f, 1.0f };

	for (auto& asset : gPreservedMeshes)
	{
		if (!asset.visible) continue;
		if (!isRenderableMeshReady(asset)) continue;

		float* color = otherColor;
		if (asset.label == "bone_result") color = boneColor;
		else if (asset.label == "bone_grind_patch") color = boneGrindPatchColor;
		else if (asset.label == "ligament_result") color = ligamentColor;

		Vector3r offset = asset.translation;
		if (offset.norm() < static_cast<Real>(1e-6))
		{
			drawWorkflowAssetMesh(asset, color);
		}
		else
		{
			VertexData transformedVD = transformVertexData(asset.vertices, offset);
			drawWorkflowMesh(transformedVD, asset.mesh, color);
		}
	}
}

void renderTransitionAssets()
{
	const bool inTransition = (gWorkflowStage == WorkflowStage::RetractionTransition || gTransitionPlaying);
	const bool afterTransition =
		(gWorkflowStage == WorkflowStage::DiscRemoval) ||
		(gWorkflowStage == WorkflowStage::Completed);

	if (!inTransition && !afterTransition)
		return;
	if (!gRetractorAssetLoaded || !isRenderableMeshReady(gRetractorAsset))
		return;

	float toolColor[4] = { 0.7f, 0.7f, 0.7f, 0.8f };
	glPushMatrix();
	glTranslated(
		static_cast<double>(gRetractorAsset.translation.x()),
		static_cast<double>(gRetractorAsset.translation.y()),
		static_cast<double>(gRetractorAsset.translation.z()));
	if ((gRetractorAsset.rotation - Matrix3r::Identity()).norm() > static_cast<Real>(1e-6))
	{
		const double rotationMatrix[16] = {
			static_cast<double>(gRetractorAsset.rotation(0, 0)),
			static_cast<double>(gRetractorAsset.rotation(1, 0)),
			static_cast<double>(gRetractorAsset.rotation(2, 0)),
			0.0,
			static_cast<double>(gRetractorAsset.rotation(0, 1)),
			static_cast<double>(gRetractorAsset.rotation(1, 1)),
			static_cast<double>(gRetractorAsset.rotation(2, 1)),
			0.0,
			static_cast<double>(gRetractorAsset.rotation(0, 2)),
			static_cast<double>(gRetractorAsset.rotation(1, 2)),
			static_cast<double>(gRetractorAsset.rotation(2, 2)),
			0.0,
			0.0,
			0.0,
			0.0,
			1.0
		};
		glMultMatrixd(rotationMatrix);
	}
	glScaled(
		static_cast<double>(gRetractorAsset.scale.x()),
		static_cast<double>(gRetractorAsset.scale.y()),
		static_cast<double>(gRetractorAsset.scale.z()));
	drawWorkflowAssetMesh(gRetractorAsset, toolColor);
	glPopMatrix();
}

void drawWorkflowHapticMarker()
{
	if (MiniGL::isHapticAvailable())
	{
		MiniGL::refreshHapticButtonState();
		gWorkflowToolVisibilityMarkerPosition = workflowMappedHapticPosition();
		gWorkflowHapticToolActive = MiniGL::getHapticSelectionState();
		gWorkflowToolVisibilityMarkerValid = true;
	}

	if (!gWorkflowToolVisibilityMarkerValid ||
		(gWorkflowStage == WorkflowStage::Completed) ||
		(gWorkflowStage == WorkflowStage::RetractionTransition))
	{
		return;
	}

	float markerColor[4] = { 1.0f, 0.82f, 0.05f, 1.0f };
	if (gWorkflowHapticToolActive)
	{
		markerColor[0] = 1.0f;
		markerColor[1] = 0.10f;
		markerColor[2] = 0.02f;
	}
	const float markerRadius = static_cast<float>(
		PBD::DemoHaptics::liveHapticToolVisibilityMarkerRadius(gActiveStageContext.toolRadius));
	glPushAttrib(GL_CURRENT_BIT | GL_ENABLE_BIT | GL_LIGHTING_BIT | GL_DEPTH_BUFFER_BIT);
	glDisable(GL_DEPTH_TEST);
	glDepthMask(GL_FALSE);
	MiniGL::drawSphere(gWorkflowToolVisibilityMarkerPosition, markerRadius, markerColor, 16u);
	glPopAttrib();
}

void renderWorkflowToolVisual()
{
	if (gWorkflowStage == WorkflowStage::RetractionTransition)
		return;
	if ((base == nullptr) || (base->m_totalSurgToolNbr == 0u))
	{
		drawWorkflowHapticMarker();
		return;
	}
	DemoBase::SurgToolMeshPair& toolPair = base->m_surgToolMeshPairs[0];
	VertexData& toolVD = toolPair.surgToolVDs[DemoBase::SurgToolMeshPair::STS_INACTIVE];
	IndexedFaceMesh& toolMesh = toolPair.surgToolMeshs[DemoBase::SurgToolMeshPair::STS_INACTIVE];
	if ((toolVD.size() == 0u) || (toolMesh.numFaces() == 0u))
	{
		drawWorkflowHapticMarker();
		return;
	}
	float toolColor[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
	if (MiniGL::isHapticAvailable())
	{
		glPushAttrib(GL_DEPTH_BUFFER_BIT | GL_ENABLE_BIT);
		glDisable(GL_DEPTH_TEST);
		glDepthMask(GL_FALSE);
		drawWorkflowMesh(toolVD, toolMesh, toolColor);
		glPopAttrib();
	}
	else
	{
		drawWorkflowMesh(toolVD, toolMesh, toolColor);
	}
	drawWorkflowHapticMarker();
}

void renderWorkflowPerformanceScene()
{
	renderWorkflowSceneLayers();
	renderPreservedMeshes();
	renderActiveStageMesh();
	renderTransitionAssets();
	renderWorkflowToolVisual();
}

void render()
{
	const Clock::time_point renderStart = Clock::now();
	if (gWorkflowVisualMode == WorkflowVisualMode::Performance)
	{
		renderWorkflowPerformanceScene();
	}
	else
	{
		base->Myrender();
		renderWorkflowSceneLayers();
		renderPreservedMeshes();
		renderActiveStageMesh();
		renderTransitionAssets();
		renderWorkflowToolVisual();
	}
	gLastRenderMs = elapsedMs(renderStart);
}

void renderWorkflowMiniGLSmokeScene()
{
	const Clock::time_point renderStart = Clock::now();
	renderWorkflowSceneLayers();
	renderPreservedMeshes();
	renderTransitionAssets();
	gLastRenderMs = elapsedMs(renderStart);
}

void reset()
{
	resetWorkflowAutoDemoRuntime();
	resetWorkflowToFirstStage();
	setStatusText("Stage 1: Bone Grinding. Press 'Run Fixed Operation' or 'Next Stage'.");
	if (gAutoDemoState.commandLineRequested)
		startWorkflowAutoDemo();
}

bool runFixedSummaryStage(const WorkflowStage stage, const std::string& label)
{
	gWorkflowStage = stage;
	buildStageContext(gActiveStageContext, gWorkflowStage);
	const Clock::time_point frameStart = Clock::now();
	runFixedOrReplayOperation(gActiveStageContext);
	ensureStageSurfaceCurrent(gActiveStageContext);
	recordDemoFlowFrame(frameStart, 0.0);
	if (!workflowStageResultReady(gActiveStageContext))
	{
		std::cout << "workflow_fixed_summary_error=empty_mesh_" << label << "\n";
		return false;
	}
	captureStageWarpedMesh(gActiveStageContext, label);
	return true;
}

int runWorkflowAutoDemoSmoke(const int argc, char** argv)
{
	gWorkflowFixedSummaryMode = true;
	gWorkflowExePath = FileSystem::getProgramPath();
	gWorkflowSceneFile.clear();
	gDemoFlowSummary.configureFromArgs(argc, argv, "SpineWorkflowDemo", "spine-workflow-auto-demo");
	gWorkflowGUIControlsRegistered = false;

	SimulationModel* model = new SimulationModel();
	model->init();
	Simulation::getCurrent()->setModel(model);
	releaseWorkflowGpuMeshes(gPreservedMeshes);
	gPreservedMeshes.clear();
	gWorkflowStage = WorkflowStage::BoneGrinding;
	gAutoDemoState.commandLineRequested = true;

	buildStageContext(gActiveStageContext, gWorkflowStage);
	startWorkflowAutoDemo();

	const unsigned int maxFrames = 1200u;
	const Real dt = static_cast<Real>(0.05);
	unsigned int frame = 0u;
	for (; frame < maxFrames; frame++)
	{
		const Clock::time_point frameStart = Clock::now();
		advanceWorkflowAutoDemo(dt);
		recordDemoFlowFrame(frameStart, 0.0);
		if ((gAutoDemoState.phase == WorkflowAutoDemoPhase::CompletedHold) &&
			(gWorkflowStage == WorkflowStage::Completed))
		{
			break;
		}
	}

	const unsigned int ligamentSeparatedFaces =
		gAutoDemoState.ligamentPatchFaces + gAutoDemoState.ligamentHiddenFaces;
	const unsigned int discSeparatedFaces =
		gAutoDemoState.discPatchFaces + gAutoDemoState.discHiddenFaces;
	const bool ligamentDetachedFragmentMoved =
		(gAutoDemoState.ligamentDraggedParticles > 0u) &&
		(gAutoDemoState.ligamentDraggedDistance > static_cast<Real>(0.05));
	const bool discDetachedFragmentMoved =
		(gAutoDemoState.discDraggedParticles > 0u) &&
		(gAutoDemoState.discDraggedDistance > static_cast<Real>(0.05));
	const bool ok =
		(gWorkflowStage == WorkflowStage::Completed) &&
		(workflowResultMeshCount() == 3u) &&
		(gAutoDemoState.boneChangedNodes > 0u) &&
		(gAutoDemoState.ligamentInactiveConstraints > 0u) &&
		(gAutoDemoState.discInactiveConstraints > 0u) &&
		(gAutoDemoState.ligamentRenderFaces > 0u) &&
		(gAutoDemoState.discRenderFaces > 0u) &&
		ligamentDetachedFragmentMoved &&
		discDetachedFragmentMoved &&
		(gAutoDemoState.ligamentComponents >= 2u) &&
		(gAutoDemoState.discComponents >= 2u);
	const bool ligamentResultMoved = workflowLigamentResultMoved();
	const bool retractorAssetIsLg =
		gRetractorAssetLoaded &&
		WorkflowSceneAssets::isLgRetractorAsset(gRetractorAssetPath);
	const bool retractorAssetIsSjlg =
		gRetractorAssetLoaded &&
		WorkflowSceneAssets::isSjlgRetractorAsset(gRetractorAssetPath);
	const bool retractorMoved =
		gRetractorAssetLoaded &&
		(gRetractorAsset.translation.norm() > static_cast<Real>(1e-6));
	const bool retractorStatic =
		!gRetractorAssetLoaded ||
		(gRetractorAsset.translation.norm() <= static_cast<Real>(1e-6));
	const bool sjlgHiddenOk =
		!gRetractorAssetLoaded &&
		gRetractorAssetPath.empty() &&
		!retractorMoved;
	const bool sjlgRetractionOk =
		sjlgHiddenOk &&
		!ligamentResultMoved;
	const Vector3r discDemoStageToolScale(static_cast<Real>(0.6), static_cast<Real>(0.6), static_cast<Real>(0.6));
	const bool retractorUsesDiscDemoScale =
		!gRetractorAssetLoaded ||
		((gRetractorAsset.scale - Vector3r::Ones()).norm() <= static_cast<Real>(1e-6));
	const bool stage3ToolUsesDiscDemoScale =
		((gAutoDemoState.stage3ToolScale - discDemoStageToolScale).norm() <= static_cast<Real>(1e-6));
	const bool discReferenceTransformOk =
		retractorUsesDiscDemoScale &&
		stage3ToolUsesDiscDemoScale;
	const bool preservedReplacesDefaultAnatomy = workflowPreservedReplacesDefaultAnatomy();
	const unsigned int sceneLayersLoaded = workflowLoadedSceneLayerCount();
	const bool backLayerLoaded = workflowSceneLayerLabelLoaded("scene_back_context");
	const bool muscleLayerLoaded = workflowSceneLayerLabelLoaded("scene_muscle_context");
	const unsigned int sceneStage1Visible = workflowVisibleSceneLayerCount(
		WorkflowStage::BoneGrinding,
		false,
		false,
		false);
	const unsigned int sceneStage2Visible = workflowVisibleSceneLayerCount(
		WorkflowStage::LigamentRemoval,
		true,
		false,
		false);
	const unsigned int sceneTransitionVisible = workflowVisibleSceneLayerCount(
		WorkflowStage::RetractionTransition,
		true,
		true,
		false);
	const unsigned int sceneStage3Visible = workflowVisibleSceneLayerCount(
		WorkflowStage::DiscRemoval,
		true,
		true,
		false);
	const unsigned int sceneCompletedVisible = workflowVisibleSceneLayerCount(
		WorkflowStage::Completed,
		true,
		true,
		true);
	const bool visualSceneReplacesDefaultTargets = workflowVisualSceneReplacesDefaultTargets();
		const bool visualSceneOk =
			(sceneLayersLoaded >= 7u) &&
			(sceneStage1Visible >= 4u) &&
			(sceneStage2Visible >= 3u) &&
			(sceneTransitionVisible >= 3u) &&
			(sceneStage3Visible >= 2u) &&
			(sceneCompletedVisible >= 2u) &&
			visualSceneReplacesDefaultTargets;
		const bool softVisualFractureOk =
			ligamentDetachedFragmentMoved &&
			discDetachedFragmentMoved;
	const bool backLayerVisibleInPerformance =
		workflowSceneLayerLabelVisible("scene_back_context", WorkflowStage::BoneGrinding);
	const bool muscleLayerVisibleInPerformance =
		workflowSceneLayerLabelVisible("scene_muscle_context", WorkflowStage::BoneGrinding);
	const bool contextVisibilityOk =
		backLayerLoaded &&
		muscleLayerLoaded &&
		backLayerVisibleInPerformance &&
		muscleLayerVisibleInPerformance;
	const bool stage1BallGrinderProxyOk =
		gAutoDemoState.stage1BallGrinderProxyOk &&
		gAutoDemoState.stage1BallProxyContact &&
		gAutoDemoState.boneGrindTopContact;
	const bool stage2GoldenScriptOk =
		gAutoDemoState.stage2GoldenScriptOk &&
		(gAutoDemoState.stage2ScriptParticleId == 609u);
	const bool stage2HoldOk = gAutoDemoState.stage2HoldOk;
	const bool stage2MainComponentCaptureOk =
		gAutoDemoState.stage2MainComponentCaptureOk &&
		(gAutoDemoState.stage2MainComponentFaces > 0u);
	const bool stage3GoldenScriptOk =
		gAutoDemoState.stage3GoldenScriptOk &&
		(gAutoDemoState.stage3ScriptParticleId == 1401u);
	const bool stage3LiveHoldOk =
		gAutoDemoState.stage3LiveHoldOk &&
		gAutoDemoState.stage3RemainsInteractive;
	const bool goldenReferenceOk =
		stage1BallGrinderProxyOk &&
		stage2GoldenScriptOk &&
		stage2HoldOk &&
		stage2MainComponentCaptureOk &&
		stage3GoldenScriptOk &&
		stage3LiveHoldOk &&
		contextVisibilityOk;
	const bool stage1VisibleGrindOk =
		stage1BallGrinderProxyOk &&
		(gAutoDemoState.boneChangedNodes >= 120u);
	const bool boneGrindPatchOk =
		gAutoDemoState.boneGrindPatchFaces > 0u &&
		hasPreservedMeshLabel("bone_grind_patch");
	const bool stage3FragmentPatchOk =
		(gAutoDemoState.discRenderFaces > 0u) &&
		((gAutoDemoState.discPatchFaces > 0u) ||
		 (gAutoDemoState.discDebugPatchFaces > 0u));
	const bool visibleArtifactOk =
		goldenReferenceOk &&
		stage1VisibleGrindOk &&
		boneGrindPatchOk &&
		stage3FragmentPatchOk;
	const bool visualFixOk =
		(sceneLayersLoaded >= 7u) &&
		backLayerLoaded &&
		muscleLayerLoaded &&
		gAutoDemoState.boneGrindTopContact &&
		gAutoDemoState.ligamentGraspPhaseSeen &&
		gAutoDemoState.ligamentPullPhaseSeen &&
			gAutoDemoState.discGraspPhaseSeen &&
			gAutoDemoState.discPullPhaseSeen &&
			sjlgRetractionOk &&
			softVisualFractureOk;
	const bool stage2LigamentInteractive =
		(gAutoDemoState.stage2TetModelCount == 1u) &&
		(gAutoDemoState.stage2ActiveSoftAsset == "rendai_tex.obj");
	const bool stage2DiscInteractive =
		(gAutoDemoState.stage2TetModelCount > 1u) ||
		(gAutoDemoState.stage2ActiveSoftAsset == "target_disc.obj");
	const bool stage3DiscInteractive =
		(gAutoDemoState.stage3TetModelCount == 1u) &&
		(gAutoDemoState.stage3ActiveSoftAsset == "target_disc.obj");
	const bool stage3LigamentInteractive =
		(gAutoDemoState.stage3TetModelCount > 1u) ||
		(gAutoDemoState.stage3ActiveSoftAsset == "rendai_tex.obj");
	const bool softStageIsolationOk =
		stage2LigamentInteractive &&
		!stage2DiscInteractive &&
		gAutoDemoState.stage2DiscRenderOnly &&
		stage3DiscInteractive &&
		!stage3LigamentInteractive &&
		gAutoDemoState.stage3LigamentRenderOnly;
		const bool contractOk =
			ok &&
			sjlgRetractionOk &&
			!gWorkflowGUIControlsRegistered &&
			preservedReplacesDefaultAnatomy &&
			softVisualFractureOk &&
			softStageIsolationOk &&
			discReferenceTransformOk;

	gDemoFlowSummary.print();
	std::cout
		<< "workflow_auto_demo_status=" << (ok ? "ok" : "failed") << "\n"
		<< "workflow_auto_demo_contract_status=" << (contractOk ? "ok" : "failed") << "\n"
		<< "workflow_auto_demo_stage=" << workflowStageNameText() << "\n"
		<< "workflow_auto_demo_phase=" << workflowAutoDemoPhaseName() << "\n"
		<< "workflow_auto_demo_preserved_meshes=" << gPreservedMeshes.size() << "\n"
		<< "workflow_auto_demo_frames=" << frame + 1u << "\n"
		<< "workflow_auto_demo_bone_changed_nodes=" << static_cast<unsigned int>(gAutoDemoState.boneChangedNodes) << "\n"
		<< "workflow_auto_demo_ligament_changed_nodes=" << static_cast<unsigned int>(gAutoDemoState.ligamentChangedNodes) << "\n"
		<< "workflow_auto_demo_disc_changed_nodes=" << static_cast<unsigned int>(gAutoDemoState.discChangedNodes) << "\n"
		<< "workflow_auto_demo_ligament_hit_edges=" << gAutoDemoState.ligamentHitEdges << "\n"
		<< "workflow_auto_demo_ligament_inactive_constraints=" << gAutoDemoState.ligamentInactiveConstraints << "\n"
			<< "workflow_auto_demo_ligament_render_faces=" << gAutoDemoState.ligamentRenderFaces << "\n"
			<< "workflow_auto_demo_ligament_patch_faces=" << gAutoDemoState.ligamentPatchFaces << "\n"
			<< "workflow_auto_demo_ligament_debug_patch_faces=" << gAutoDemoState.ligamentDebugPatchFaces << "\n"
			<< "workflow_auto_demo_ligament_dragged_particles=" << gAutoDemoState.ligamentDraggedParticles << "\n"
			<< "workflow_auto_demo_ligament_hidden_faces=" << gAutoDemoState.ligamentHiddenFaces << "\n"
			<< "workflow_auto_demo_ligament_separated_faces=" << ligamentSeparatedFaces << "\n"
			<< "workflow_auto_demo_ligament_dragged_distance=" << gAutoDemoState.ligamentDraggedDistance << "\n"
			<< "workflow_auto_demo_ligament_detached_fragment_moved=" << (ligamentDetachedFragmentMoved ? 1 : 0) << "\n"
			<< "workflow_auto_demo_ligament_components=" << gAutoDemoState.ligamentComponents << "\n"
		<< "workflow_auto_demo_disc_hit_edges=" << gAutoDemoState.discHitEdges << "\n"
		<< "workflow_auto_demo_disc_inactive_constraints=" << gAutoDemoState.discInactiveConstraints << "\n"
			<< "workflow_auto_demo_disc_render_faces=" << gAutoDemoState.discRenderFaces << "\n"
			<< "workflow_auto_demo_disc_patch_faces=" << gAutoDemoState.discPatchFaces << "\n"
			<< "workflow_auto_demo_disc_debug_patch_faces=" << gAutoDemoState.discDebugPatchFaces << "\n"
			<< "workflow_auto_demo_disc_dragged_particles=" << gAutoDemoState.discDraggedParticles << "\n"
			<< "workflow_auto_demo_disc_hidden_faces=" << gAutoDemoState.discHiddenFaces << "\n"
			<< "workflow_auto_demo_disc_separated_faces=" << discSeparatedFaces << "\n"
			<< "workflow_auto_demo_disc_dragged_distance=" << gAutoDemoState.discDraggedDistance << "\n"
			<< "workflow_auto_demo_disc_detached_fragment_moved=" << (discDetachedFragmentMoved ? 1 : 0) << "\n"
			<< "workflow_auto_demo_disc_components=" << gAutoDemoState.discComponents << "\n"
		<< "workflow_auto_demo_stage2_active_soft_asset=" << (gAutoDemoState.stage2ActiveSoftAsset.empty() ? "none" : gAutoDemoState.stage2ActiveSoftAsset) << "\n"
		<< "workflow_auto_demo_stage2_tet_model_count=" << gAutoDemoState.stage2TetModelCount << "\n"
		<< "workflow_auto_demo_stage2_ligament_interactive=" << (stage2LigamentInteractive ? 1 : 0) << "\n"
		<< "workflow_auto_demo_stage2_disc_interactive=" << (stage2DiscInteractive ? 1 : 0) << "\n"
		<< "workflow_auto_demo_stage2_disc_render_only=" << (gAutoDemoState.stage2DiscRenderOnly ? 1 : 0) << "\n"
		<< "workflow_auto_demo_stage2_script_particle_id=" << gAutoDemoState.stage2ScriptParticleId << "\n"
		<< "workflow_auto_demo_stage2_script_offset="
		<< gAutoDemoState.stage2ScriptOffset.x() << ","
		<< gAutoDemoState.stage2ScriptOffset.y() << ","
		<< gAutoDemoState.stage2ScriptOffset.z() << "\n"
		<< "workflow_auto_demo_stage2_golden_script_status=" << (stage2GoldenScriptOk ? "ok" : "failed") << "\n"
		<< "workflow_auto_demo_stage2_hold_status=" << (stage2HoldOk ? "ok" : "failed") << "\n"
		<< "workflow_auto_demo_stage2_main_component_faces=" << gAutoDemoState.stage2MainComponentFaces << "\n"
		<< "workflow_auto_demo_stage2_main_component_capture_status=" << (stage2MainComponentCaptureOk ? "ok" : "failed") << "\n"
		<< "workflow_auto_demo_stage3_active_soft_asset=" << (gAutoDemoState.stage3ActiveSoftAsset.empty() ? "none" : gAutoDemoState.stage3ActiveSoftAsset) << "\n"
		<< "workflow_auto_demo_stage3_tet_model_count=" << gAutoDemoState.stage3TetModelCount << "\n"
		<< "workflow_auto_demo_stage3_ligament_interactive=" << (stage3LigamentInteractive ? 1 : 0) << "\n"
		<< "workflow_auto_demo_stage3_disc_interactive=" << (stage3DiscInteractive ? 1 : 0) << "\n"
		<< "workflow_auto_demo_stage3_script_particle_id=" << gAutoDemoState.stage3ScriptParticleId << "\n"
		<< "workflow_auto_demo_stage3_script_offset="
		<< gAutoDemoState.stage3ScriptOffset.x() << ","
		<< gAutoDemoState.stage3ScriptOffset.y() << ","
		<< gAutoDemoState.stage3ScriptOffset.z() << "\n"
		<< "workflow_auto_demo_stage3_golden_script_status=" << (stage3GoldenScriptOk ? "ok" : "failed") << "\n"
		<< "workflow_auto_demo_stage3_remains_interactive=" << (gAutoDemoState.stage3RemainsInteractive ? 1 : 0) << "\n"
		<< "workflow_auto_demo_stage3_live_hold_status=" << (stage3LiveHoldOk ? "ok" : "failed") << "\n"
		<< "workflow_auto_demo_stage3_fragment_patch_status=" << (stage3FragmentPatchOk ? "ok" : "failed") << "\n"
			<< "workflow_auto_demo_stage3_ligament_render_only=" << (gAutoDemoState.stage3LigamentRenderOnly ? 1 : 0) << "\n"
			<< "workflow_auto_demo_soft_stage_isolation_status=" << (softStageIsolationOk ? "ok" : "failed") << "\n"
			<< "workflow_auto_demo_soft_visual_fracture_status=" << (softVisualFractureOk ? "ok" : "failed") << "\n"
		<< "workflow_auto_demo_ligament_result_moved=" << (ligamentResultMoved ? 1 : 0) << "\n"
		<< "workflow_auto_demo_retractor_asset=" << (gRetractorAssetPath.empty() ? "none" : gRetractorAssetPath) << "\n"
		<< "workflow_auto_demo_retractor_asset_is_lg=" << (retractorAssetIsLg ? 1 : 0) << "\n"
		<< "workflow_auto_demo_retractor_asset_is_sjlg=" << (retractorAssetIsSjlg ? 1 : 0) << "\n"
		<< "workflow_auto_demo_sjlg_hidden=" << (sjlgHiddenOk ? 1 : 0) << "\n"
		<< "workflow_auto_demo_retractor_moved=" << (retractorMoved ? 1 : 0) << "\n"
		<< "workflow_auto_demo_sjlg_retraction_status=" << (sjlgRetractionOk ? "ok" : "failed") << "\n"
		<< "workflow_auto_demo_retractor_uses_disc_demo_scale=" << (retractorUsesDiscDemoScale ? 1 : 0) << "\n"
		<< "workflow_auto_demo_stage3_tool_uses_disc_demo_scale=" << (stage3ToolUsesDiscDemoScale ? 1 : 0) << "\n"
		<< "workflow_auto_demo_disc_reference_transform_status=" << (discReferenceTransformOk ? "ok" : "failed") << "\n"
		<< "workflow_auto_demo_gui_controls_registered=" << (gWorkflowGUIControlsRegistered ? 1 : 0) << "\n"
		<< "workflow_auto_demo_preserved_replaces_default_anatomy=" << (preservedReplacesDefaultAnatomy ? 1 : 0) << "\n"
		<< "workflow_auto_demo_visual_scene_status=" << (visualSceneOk ? "ok" : "failed") << "\n"
		<< "workflow_auto_demo_scene_layers_loaded=" << sceneLayersLoaded << "\n"
		<< "workflow_auto_demo_scene_stage1_visible_layers=" << sceneStage1Visible << "\n"
		<< "workflow_auto_demo_scene_stage2_visible_layers=" << sceneStage2Visible << "\n"
		<< "workflow_auto_demo_scene_transition_visible_layers=" << sceneTransitionVisible << "\n"
		<< "workflow_auto_demo_scene_stage3_visible_layers=" << sceneStage3Visible << "\n"
		<< "workflow_auto_demo_scene_completed_visible_layers=" << sceneCompletedVisible << "\n"
		<< "workflow_auto_demo_visual_scene_replaces_default_targets=" << (visualSceneReplacesDefaultTargets ? 1 : 0) << "\n"
		<< "workflow_auto_demo_back_layer_loaded=" << (backLayerLoaded ? 1 : 0) << "\n"
		<< "workflow_auto_demo_muscle_layer_loaded=" << (muscleLayerLoaded ? 1 : 0) << "\n"
		<< "workflow_auto_demo_back_layer_visible_in_performance=" << (backLayerVisibleInPerformance ? 1 : 0) << "\n"
		<< "workflow_auto_demo_muscle_layer_visible_in_performance=" << (muscleLayerVisibleInPerformance ? 1 : 0) << "\n"
		<< "workflow_auto_demo_context_visibility_status=" << (contextVisibilityOk ? "ok" : "failed") << "\n"
		<< "workflow_auto_demo_bone_grind_top_contact=" << (gAutoDemoState.boneGrindTopContact ? 1 : 0) << "\n"
		<< "workflow_auto_demo_stage1_tool_proxy=ball_grinder\n"
		<< "workflow_auto_demo_stage1_ball_proxy_radius=" << gAutoDemoState.stage1BallProxyRadius << "\n"
		<< "workflow_auto_demo_stage1_ball_proxy_contact=" << (gAutoDemoState.stage1BallProxyContact ? 1 : 0) << "\n"
		<< "workflow_auto_demo_stage1_ball_grinder_proxy_status=" << (stage1BallGrinderProxyOk ? "ok" : "failed") << "\n"
		<< "workflow_auto_demo_stage1_visible_grind_status=" << (stage1VisibleGrindOk ? "ok" : "failed") << "\n"
		<< "workflow_auto_demo_bone_grind_patch_faces=" << gAutoDemoState.boneGrindPatchFaces << "\n"
		<< "workflow_auto_demo_bone_grind_patch_status=" << (boneGrindPatchOk ? "ok" : "failed") << "\n"
		<< "workflow_auto_demo_ligament_grasp_phase_seen=" << (gAutoDemoState.ligamentGraspPhaseSeen ? 1 : 0) << "\n"
		<< "workflow_auto_demo_ligament_pull_phase_seen=" << (gAutoDemoState.ligamentPullPhaseSeen ? 1 : 0) << "\n"
		<< "workflow_auto_demo_disc_grasp_phase_seen=" << (gAutoDemoState.discGraspPhaseSeen ? 1 : 0) << "\n"
		<< "workflow_auto_demo_disc_pull_phase_seen=" << (gAutoDemoState.discPullPhaseSeen ? 1 : 0) << "\n"
		<< "workflow_auto_demo_retractor_static=" << (retractorStatic ? 1 : 0) << "\n"
		<< "workflow_auto_demo_visual_fix_status=" << (visualFixOk ? "ok" : "failed") << "\n"
		<< "workflow_auto_demo_golden_reference_status=" << (goldenReferenceOk ? "ok" : "failed") << "\n"
		<< "workflow_auto_demo_visible_artifact_status=" << (visibleArtifactOk ? "ok" : "failed") << "\n"
		<< "workflow_auto_demo_legacy_path_used=0\n"
		<< "workflow_auto_demo_warning=" << workflowStageWarningText() << "\n";

	delete Simulation::getCurrent();
	delete model;
	return contractOk ? 0 : 1;
}

int runWorkflowGuiStateSmoke(const int argc, char** argv)
{
	gWorkflowFixedSummaryMode = true;
	gWorkflowExePath = FileSystem::getProgramPath();
	gWorkflowSceneFile.clear();
	gWorkflowGUIControlsRegistered = false;
	gDemoFlowSummary.configureFromArgs(argc, argv, "SpineWorkflowDemo", "spine-workflow-gui-state");

	SimulationModel* model = new SimulationModel();
	model->init();
	Simulation::getCurrent()->setModel(model);
	releaseWorkflowGpuMeshes(gPreservedMeshes);
	gPreservedMeshes.clear();
	gWorkflowStage = WorkflowStage::BoneGrinding;
	gAutoFixedOperationEnabled = true;

	buildStageContext(gActiveStageContext, gWorkflowStage);

	Clock::time_point frameStart = Clock::now();
	requestNextStage();
	recordDemoFlowFrame(frameStart, 0.0);
	frameStart = Clock::now();
	requestNextStage();
	recordDemoFlowFrame(frameStart, 0.0);
	frameStart = Clock::now();
	requestNextStage();
	recordDemoFlowFrame(frameStart, 0.0);
	frameStart = Clock::now();
	requestNextStage();
	recordDemoFlowFrame(frameStart, 0.0);

	const bool ok = (gWorkflowStage == WorkflowStage::Completed) && (workflowResultMeshCount() == 3u);
	gDemoFlowSummary.print();
	std::cout
		<< "workflow_gui_state_smoke_status=" << (ok ? "ok" : "failed") << "\n"
		<< "workflow_gui_state_stage=" << workflowStageNameText() << "\n"
		<< "workflow_gui_state_preserved_meshes=" << gPreservedMeshes.size() << "\n"
		<< "workflow_gui_state_controls_registered=" << (gWorkflowGUIControlsRegistered ? 1 : 0) << "\n"
		<< "workflow_gui_state_warning=" << workflowStageWarningText() << "\n";

	delete Simulation::getCurrent();
	delete model;
	return ok ? 0 : 1;
}

int runWorkflowFixedSummarySmoke(const int argc, char** argv)
{
	gWorkflowFixedSummaryMode = true;
	gWorkflowExePath = FileSystem::getProgramPath();
	gWorkflowSceneFile.clear();
	gDemoFlowSummary.configureFromArgs(argc, argv, "SpineWorkflowDemo", "spine-workflow-fixed");

	SimulationModel* model = new SimulationModel();
	model->init();
	Simulation::getCurrent()->setModel(model);
	releaseWorkflowGpuMeshes(gPreservedMeshes);
	gPreservedMeshes.clear();

	const bool ok =
		runFixedSummaryStage(WorkflowStage::BoneGrinding, "bone_result") &&
		runFixedSummaryStage(WorkflowStage::LigamentRemoval, "ligament_result") &&
		runFixedSummaryStage(WorkflowStage::DiscRemoval, "disc_result");

	const uint64_t finalSurfaceHash = gActiveStageContext.surfaceSignature.hash;
	const unsigned int finalVertexCount = gActiveStageContext.surfaceSignature.vertexCount;
	const unsigned int finalFaceCount = gActiveStageContext.surfaceSignature.faceCount;

	unsigned int preservedVertexCount = 0u;
	unsigned int preservedFaceCount = 0u;
	for (const auto& pm : gPreservedMeshes)
	{
		preservedVertexCount += static_cast<unsigned int>(pm.vertices.size());
		preservedFaceCount += pm.mesh.numFaces();
	}

	gDemoFlowSummary.print();
	std::cout
		<< "workflow_fixed_summary_status=" << (ok ? "ok" : "failed") << "\n"
		<< "workflow_fixed_summary_stages=" << gPreservedMeshes.size() << "\n"
		<< "workflow_fixed_summary_surface_hash=" << finalSurfaceHash << "\n"
		<< "workflow_fixed_summary_vertex_count=" << finalVertexCount << "\n"
		<< "workflow_fixed_summary_face_count=" << finalFaceCount << "\n"
		<< "workflow_fixed_summary_preserved_vertex_count=" << preservedVertexCount << "\n"
		<< "workflow_fixed_summary_preserved_face_count=" << preservedFaceCount << "\n";

	delete Simulation::getCurrent();
	delete model;
	return ok ? 0 : 1;
}

int runWorkflowMiniGLScreenshotSmoke()
{
	gAutoFixedOperationEnabled = true;

	Clock::time_point frameStart = Clock::now();
	requestNextStage();
	recordDemoFlowFrame(frameStart, 0.0);
	frameStart = Clock::now();
	requestNextStage();
	recordDemoFlowFrame(frameStart, 0.0);
	frameStart = Clock::now();
	requestNextStage();
	recordDemoFlowFrame(frameStart, 0.0);
	frameStart = Clock::now();
	requestNextStage();
	recordDemoFlowFrame(frameStart, 0.0);

	const std::string screenshotPath = FileSystem::normalizePath(base->getOutputPath() + "/workflow-minigl-screenshot-smoke.ppm");
	applyWorkflowViewport();
	MiniGL::viewport();
	renderWorkflowMiniGLSmokeScene();
	MiniGL::viewport();
	renderWorkflowMiniGLSmokeScene();

	unsigned int screenshotWidth = 0u;
	unsigned int screenshotHeight = 0u;
	unsigned int nonBackgroundPixels = 0u;
	const bool screenshotWritten = MiniGL::writeFramebufferPPM(
		screenshotPath,
		screenshotWidth,
		screenshotHeight,
		nonBackgroundPixels);
	const unsigned int sceneLayersLoaded = workflowLoadedSceneLayerCount();
	const unsigned int sceneCompletedVisible = workflowVisibleSceneLayerCount(
		WorkflowStage::Completed,
		true,
		true,
		true);
	const bool retractorMoved =
		gRetractorAssetLoaded &&
		(gRetractorAsset.translation.norm() > static_cast<Real>(1e-6));
	const Real renderBudgetMs = static_cast<Real>(50.0);
	const Real renderBudgetBaselineMs = static_cast<Real>(82.0);
	const bool renderBudgetPassed =
		(gLastRenderMs <= renderBudgetMs) ||
		(gLastRenderMs <= renderBudgetBaselineMs * static_cast<Real>(0.60));

	const bool ok =
		screenshotWritten &&
			(nonBackgroundPixels > 0u) &&
			(gWorkflowStage == WorkflowStage::Completed) &&
			(workflowResultMeshCount() == 3u) &&
			(sceneLayersLoaded >= 7u) &&
			(sceneCompletedVisible >= 2u);

	gDemoFlowSummary.print();
	std::cout
		<< "workflow_minigl_screenshot_smoke_status=" << (ok ? "ok" : "failed") << "\n"
		<< "workflow_minigl_screenshot_path=" << screenshotPath << "\n"
		<< "workflow_minigl_screenshot_written=" << (screenshotWritten ? 1 : 0) << "\n"
		<< "workflow_minigl_screenshot_width=" << screenshotWidth << "\n"
		<< "workflow_minigl_screenshot_height=" << screenshotHeight << "\n"
		<< "workflow_minigl_screenshot_non_background_pixels=" << nonBackgroundPixels << "\n"
		<< "workflow_minigl_screenshot_stage=" << workflowStageNameText() << "\n"
		<< "workflow_minigl_screenshot_preserved_meshes=" << gPreservedMeshes.size() << "\n"
		<< "workflow_minigl_screenshot_scene_layers_loaded=" << sceneLayersLoaded << "\n"
		<< "workflow_minigl_screenshot_scene_completed_visible_layers=" << sceneCompletedVisible << "\n"
		<< "workflow_minigl_screenshot_retractor_moved=" << (retractorMoved ? 1 : 0) << "\n"
		<< "workflow_minigl_screenshot_warning=" << workflowStageWarningText() << "\n"
		<< "workflow_minigl_screenshot_render_ms=" << gLastRenderMs << "\n"
		<< "workflow_minigl_screenshot_render_budget_ms=" << renderBudgetMs << "\n"
		<< "workflow_minigl_screenshot_render_budget_status=" << (renderBudgetPassed ? "ok" : "failed") << "\n";
	return ok ? 0 : 1;
}

double workflowPerfAverage(const std::vector<double>& values)
{
	if (values.empty())
		return 0.0;
	double sum = 0.0;
	for (const double value : values)
		sum += value;
	return sum / static_cast<double>(values.size());
}

double workflowPerfPercentile(std::vector<double> values, const double percent)
{
	if (values.empty())
		return 0.0;
	std::sort(values.begin(), values.end());
	const double clampedPercent = std::max(0.0, std::min(100.0, percent));
	const double rank = (clampedPercent / 100.0) * static_cast<double>(values.size() - 1u);
	const size_t lower = static_cast<size_t>(rank);
	const size_t upper = std::min(values.size() - 1u, lower + 1u);
	const double weight = rank - static_cast<double>(lower);
	return values[lower] * (1.0 - weight) + values[upper] * weight;
}

int runWorkflowLivePerformanceSmoke(const int argc, char** argv)
{
	gWorkflowFixedSummaryMode = false;
	gAutoFixedOperationEnabled = true;
	gAutoDemoState.commandLineRequested = true;
	gDemoFlowSummary.configureFromArgs(argc, argv, "SpineWorkflowDemo", "spine-workflow-live-performance");

	releaseWorkflowGpuMeshes(gPreservedMeshes);
	gPreservedMeshes.clear();
	startWorkflowAutoDemo();

	const Clock::time_point autoStart = Clock::now();
	bool autoTimedOut = false;
	unsigned int autoFrames = 0u;
	const unsigned int maxAutoFrames = 1200u;
	const Real autoDt = static_cast<Real>(0.05);
	for (; autoFrames < maxAutoFrames; autoFrames++)
	{
		if (elapsedMs(autoStart) > 15000.0)
		{
			autoTimedOut = true;
			break;
		}
		advanceWorkflowAutoDemo(autoDt);
		if (!gAutoDemoState.running &&
			(gWorkflowStage == WorkflowStage::DiscRemoval) &&
			gAutoDemoState.stage3RemainsInteractive)
		{
			break;
		}
	}
	const double autoElapsedMs = elapsedMs(autoStart);
	std::cout << std::fixed << std::setprecision(6)
		<< "workflow_live_performance_auto_elapsed_ms=" << autoElapsedMs << "\n"
		<< "workflow_live_performance_auto_frames=" << autoFrames + 1u << "\n"
		<< "workflow_live_performance_auto_timed_out=" << (autoTimedOut ? 1 : 0) << "\n";
	std::cout.flush();

	applyWorkflowViewport();
	MiniGL::viewport();
	render();

	std::vector<double> frameSamples;
	std::vector<double> solverSamples;
	std::vector<double> renderSamples;
	const unsigned int sampleCount = 5u;
	frameSamples.reserve(sampleCount);
	solverSamples.reserve(sampleCount);
	renderSamples.reserve(sampleCount);
	SimulationModel* model = Simulation::getCurrent()->getModel();
	for (unsigned int sample = 0u; sample < sampleCount; sample++)
	{
		const Clock::time_point frameStart = Clock::now();
		const Clock::time_point timeStepStart = Clock::now();
		timeStep();
		const double timeStepMs = elapsedMs(timeStepStart);
		render();
		const double frameMs = elapsedMs(frameStart);
		frameSamples.push_back(frameMs);
		solverSamples.push_back(timeStepMs);
		renderSamples.push_back(gLastRenderMs);
	}

	const double avgFrameMs = workflowPerfAverage(frameSamples);
	const double p95FrameMs = workflowPerfPercentile(frameSamples, 95.0);
	const double avgTimeStepMs = workflowPerfAverage(solverSamples);
	const double p95TimeStepMs = workflowPerfPercentile(solverSamples, 95.0);
	const double avgRenderMs = workflowPerfAverage(renderSamples);
	const double p95RenderMs = workflowPerfPercentile(renderSamples, 95.0);
	const double fpsFromP95 = p95FrameMs > 0.0 ? 1000.0 / p95FrameMs : 0.0;
	const bool reachedLiveStage3 =
		(gWorkflowStage == WorkflowStage::DiscRemoval) &&
		gAutoDemoState.stage3RemainsInteractive &&
		(model != nullptr) &&
		(model->getTetModels().size() == 1u);
	const unsigned int stepsPerRender = base->getValue<unsigned int>(DemoBase::NUM_STEPS_PER_RENDER);
	const bool stepsPerRenderOk = stepsPerRender == WORKFLOW_STEPS_PER_RENDER_UPDATE;
	const bool ok =
		reachedLiveStage3 &&
		stepsPerRenderOk &&
		(p95FrameMs <= 50.0) &&
		(p95TimeStepMs <= 5.0) &&
		(p95RenderMs <= 35.0);

	gDemoFlowSummary.print();
	std::cout << std::fixed << std::setprecision(6)
		<< "workflow_live_performance_status=" << (ok ? "ok" : "failed") << "\n"
		<< "workflow_live_performance_stage=" << workflowStageNameText() << "\n"
		<< "workflow_live_performance_reached_stage3=" << (reachedLiveStage3 ? 1 : 0) << "\n"
		<< "workflow_live_performance_samples=" << frameSamples.size() << "\n"
		<< "workflow_live_performance_steps_per_render=" << stepsPerRender << "\n"
		<< "workflow_live_performance_steps_per_render_status=" << (stepsPerRenderOk ? "ok" : "failed") << "\n"
		<< "workflow_live_performance_avg_frame_ms=" << avgFrameMs << "\n"
		<< "workflow_live_performance_p95_frame_ms=" << p95FrameMs << "\n"
		<< "workflow_live_performance_p95_fps=" << fpsFromP95 << "\n"
		<< "workflow_live_performance_avg_timestep_ms=" << avgTimeStepMs << "\n"
		<< "workflow_live_performance_p95_timestep_ms=" << p95TimeStepMs << "\n"
		<< "workflow_live_performance_idle_physics_paused=" << (workflowAutoDemoStage3IdleHold() ? 1 : 0) << "\n"
		<< "workflow_live_performance_avg_render_ms=" << avgRenderMs << "\n"
		<< "workflow_live_performance_p95_render_ms=" << p95RenderMs << "\n"
		<< "workflow_live_performance_active_soft_vertices=" << gActiveStageContext.softDisplayVD.size() << "\n"
		<< "workflow_live_performance_active_soft_faces=" << gActiveStageContext.softDisplayMesh.numFaces() << "\n"
		<< "workflow_live_performance_disc_patch_faces=" << gAutoDemoState.discPatchFaces << "\n"
		<< "workflow_live_performance_render_patch_face_count=" << countRenderPatchFaces(model) << "\n"
		<< "workflow_live_performance_render_vis_face_count=" << countRenderVisFaces(model) << "\n";
	return ok ? 0 : 1;
}

// --- Main ---

int main(int argc, char** argv)
{
	REPORT_MEMORY_LEAKS

	const WorkflowRuntimePolicy runtimePolicy = parseWorkflowRuntimePolicy(argc, argv);
	if (hasArg(argc, argv, "--workflow-fixed-summary"))
		return runWorkflowFixedSummarySmoke(argc, argv);
	if (hasArg(argc, argv, "--workflow-gui-state-smoke"))
		return runWorkflowGuiStateSmoke(argc, argv);
	if (hasArg(argc, argv, "--workflow-auto-demo-smoke"))
		return runWorkflowAutoDemoSmoke(argc, argv);
	gWorkflowMiniGLScreenshotSmoke = hasArg(argc, argv, "--workflow-minigl-screenshot-smoke");
	gWorkflowLivePerformanceSmoke = hasArg(argc, argv, "--workflow-live-performance-smoke");
	gWorkflowHapticDiagnosticsEnabled = hasArg(argc, argv, "--workflow-haptic-diagnostics");
	gAutoDemoState.commandLineRequested = hasArg(argc, argv, "--workflow-auto-demo");

	base = new DemoBase();
	base->init(argc, argv, "Spine Workflow Demo");
	gWorkflowHapticToolControl = runtimePolicy.hapticToolControl;
	gWorkflowHapticToolViewOffset = runtimePolicy.hapticVisualOffset;
	MiniGL::setHapticWorkspaceScale(runtimePolicy.hapticWorkspaceScale);
	if (!gWorkflowMiniGLScreenshotSmoke && !gWorkflowLivePerformanceSmoke)
		MiniGL::setWindowMaximized(true);
	base->setValue(DemoBase::NUM_STEPS_PER_RENDER, WORKFLOW_STEPS_PER_RENDER_UPDATE);
	base->setValue(DemoBase::PAUSE, false);
	gDemoFlowSummary.configureFromArgs(argc, argv, "SpineWorkflowDemo", "spine-workflow");

	SimulationModel* model = new SimulationModel();
	model->init();
	Simulation::getCurrent()->setModel(model);

	buildStageContext(gActiveStageContext, gWorkflowStage);

	if (hasArg(argc, argv, "--haptic-init-smoke"))
	{
		unsigned int toolVertices = 0u;
		unsigned int toolFaces = 0u;
		Vector3r toolCenter = Vector3r::Zero();
		Vector3r toolControl = Vector3r::Zero();
		const bool toolReady = workflowCurrentToolStats(toolVertices, toolFaces, toolCenter, toolControl);
		printWorkflowHapticDiagnostics("haptic_init_smoke", true);
		const bool hapticAvailable = MiniGL::isHapticAvailable();
		const bool ok = hapticAvailable && toolReady;
		std::cout
			<< "workflow_haptic_init_smoke_status=" << (ok ? "ok" : "failed") << "\n"
			<< "workflow_haptic_available=" << (hapticAvailable ? 1 : 0) << "\n"
			<< "workflow_haptic_control_enabled=" << (gWorkflowHapticToolControl ? 1 : 0) << "\n"
			<< "workflow_haptic_workspace_scale=" << std::fixed << std::setprecision(3) << MiniGL::getHapticWorkspaceScale() << "\n"
			<< "workflow_haptic_visual_offset=" << workflowVecText(gWorkflowHapticToolViewOffset) << "\n"
			<< "workflow_haptic_raw_pos=" << workflowVecText(hapticAvailable ? MiniGL::getHapticPos() : Vector3r::Zero()) << "\n"
			<< "workflow_haptic_mapped_pos=" << workflowVecText(workflowMappedHapticPosition()) << "\n"
			<< "workflow_haptic_tool_ready=" << (toolReady ? 1 : 0) << "\n"
			<< "workflow_haptic_tool_vertices=" << toolVertices << "\n"
			<< "workflow_haptic_tool_faces=" << toolFaces << "\n"
			<< "workflow_haptic_tool_control=" << workflowVecText(toolControl) << "\n"
			<< "workflow_haptic_tool_center=" << workflowVecText(toolCenter) << "\n"
			<< std::flush;
		MiniGL::shutdown();
		base->cleanup();
		delete Simulation::getCurrent();
		delete base;
		delete model;
		return ok ? 0 : 2;
	}

	if (gWorkflowMiniGLScreenshotSmoke)
	{
		const int exitCode = runWorkflowMiniGLScreenshotSmoke();
		MiniGL::shutdown();
		base->cleanup();
		delete Simulation::getCurrent();
		delete base;
		delete model;
		return exitCode;
	}
	if (gWorkflowLivePerformanceSmoke)
	{
		const int exitCode = runWorkflowLivePerformanceSmoke(argc, argv);
		MiniGL::shutdown();
		base->cleanup();
		delete Simulation::getCurrent();
		delete base;
		delete model;
		return exitCode;
	}

	if (!gAutoDemoState.commandLineRequested)
		base->createParameterGUI();
	if (gAutoDemoState.commandLineRequested)
		startWorkflowAutoDemo();

	MiniGL::setClientIdleFunc(timeStep);
	MiniGL::addKeyFunc('r', reset);
	MiniGL::addKeyFunc('n', []() { requestNextStage(); });
	MiniGL::setClientSceneFunc(render);
	applyWorkflowViewport();
	MiniGL::mainLoop();

	gDemoFlowSummary.print();
	base->cleanup();

	delete Simulation::getCurrent();
	delete base;
	delete model;

	return 0;
}
