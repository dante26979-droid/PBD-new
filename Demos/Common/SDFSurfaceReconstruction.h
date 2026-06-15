#ifndef __SDF_SURFACE_RECONSTRUCTION_H__
#define __SDF_SURFACE_RECONSTRUCTION_H__

#include "Common/Common.h"
#include "Demos/Visualization/MC.h"
#include "Simulation/ParticleData.h"
#include "Utils/IndexedFaceMesh.h"
#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace PBD
{
	struct SDFSurfaceSignature
	{
		unsigned int vertexCount = 0u;
		unsigned int faceCount = 0u;
		uint64_t hash = 0u;
	};

	struct SDFSurfaceReconstructionStats
	{
		bool success = false;
		bool reconstructed = false;
		double marchingCubeMs = 0.0;
		double meshConvertMs = 0.0;
		unsigned int vertexCount = 0u;
		unsigned int faceCount = 0u;
		uint64_t surfaceHash = 0u;
		SDFSurfaceSignature signature;
		std::string error;
	};

	namespace SDFSurfaceReconstruction
	{
		using Clock = std::chrono::high_resolution_clock;

		inline double elapsedMs(const Clock::time_point& start)
		{
			return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
		}

		inline void hashBytes(uint64_t& hash, const void* data, const size_t size)
		{
			const unsigned char* bytes = static_cast<const unsigned char*>(data);
			for (size_t i = 0; i < size; ++i)
			{
				hash ^= static_cast<uint64_t>(bytes[i]);
				hash *= 1099511628211ull;
			}
		}

		template <typename T>
		inline void hashValue(uint64_t& hash, const T& value)
		{
			hashBytes(hash, &value, sizeof(T));
		}

		inline SDFSurfaceSignature computeSignature(const MC::mcMesh& mesh)
		{
			SDFSurfaceSignature signature;
			signature.vertexCount = static_cast<unsigned int>(mesh.vertices.size());
			signature.faceCount = static_cast<unsigned int>(mesh.indices.size() / 3u);

			uint64_t hash = 14695981039346656037ull;
			hashValue(hash, static_cast<uint64_t>(signature.vertexCount));
			hashValue(hash, static_cast<uint64_t>(signature.faceCount));
			for (const MC::mcVec3f& vertex : mesh.vertices)
			{
				const double x = static_cast<double>(vertex.x);
				const double y = static_cast<double>(vertex.y);
				const double z = static_cast<double>(vertex.z);
				hashValue(hash, x);
				hashValue(hash, y);
				hashValue(hash, z);
			}
			for (const MC::muint index : mesh.indices)
				hashValue(hash, static_cast<uint64_t>(index));
			signature.hash = hash;
			return signature;
		}

		inline bool shouldReconstruct(const bool meshUpdated, const SDFSurfaceSignature& currentSignature)
		{
			return meshUpdated || currentSignature.vertexCount == 0u || currentSignature.faceCount == 0u;
		}

		template <typename ResolutionDerived>
		inline SDFSurfaceReconstructionStats reconstruct(
			std::vector<Real>& nodeSDFVals,
			const Eigen::MatrixBase<ResolutionDerived>& resolutionSDF,
			const Eigen::AlignedBox3d& gridDomain,
			MC::mcMesh& mcMesh,
			MC::MarchingCubeWorkspace& workspace,
			VertexData& warpedVD,
			Utilities::IndexedFaceMesh& warpedMesh)
		{
			SDFSurfaceReconstructionStats stats;
			if (nodeSDFVals.empty())
			{
				stats.error = "SDF node values are empty.";
				return stats;
			}

			const Clock::time_point marchingCubeStart = Clock::now();
			MC::marching_cube(
				nodeSDFVals.data(),
				resolutionSDF[0],
				resolutionSDF[1],
				resolutionSDF[2],
				mcMesh,
				workspace);
			stats.marchingCubeMs = elapsedMs(marchingCubeStart);

			if (mcMesh.normals.size() != mcMesh.vertices.size())
			{
				stats.error = "Marching cube output normals and vertices are out of sync.";
				return stats;
			}

			MC::setDefaultArraySizes(
				static_cast<MC::muint>(std::max<size_t>(1u, mcMesh.vertices.size())),
				static_cast<MC::muint>(std::max<size_t>(1u, mcMesh.normals.size())),
				static_cast<MC::muint>(std::max<size_t>(1u, mcMesh.indices.size())));

			const unsigned int nPoints = static_cast<unsigned int>(mcMesh.vertices.size());
			const unsigned int nFaces = static_cast<unsigned int>(mcMesh.indices.size() / 3u);
			const Eigen::Vector3d& minPos = gridDomain.min();
			const Eigen::Vector3d& maxPos = gridDomain.max();

			const Real dx = static_cast<Real>((maxPos[0] - minPos[0]) / static_cast<Real>(resolutionSDF[0] - 1u));
			const Real dy = static_cast<Real>((maxPos[1] - minPos[1]) / static_cast<Real>(resolutionSDF[1] - 1u));
			const Real dz = static_cast<Real>((maxPos[2] - minPos[2]) / static_cast<Real>(resolutionSDF[2] - 1u));

			const Clock::time_point meshConvertStart = Clock::now();
			warpedVD.release();
			warpedVD.reserve(mcMesh.vertices.size());
			for (size_t i = 0; i < mcMesh.vertices.size(); i++)
			{
				warpedVD.addVertex(Vector3r(
					static_cast<Real>(minPos[0]) + static_cast<Real>(mcMesh.vertices.at(i).x) * dx,
					static_cast<Real>(minPos[1]) + static_cast<Real>(mcMesh.vertices.at(i).y) * dy,
					static_cast<Real>(minPos[2]) + static_cast<Real>(mcMesh.vertices.at(i).z) * dz));
			}

			warpedMesh.release();
			warpedMesh.initMesh(nPoints, nFaces * 2u, nFaces);
			for (size_t i = 0; i < mcMesh.indices.size(); i += 3u)
			{
				int posIndices[3];
				posIndices[0] = static_cast<int>(mcMesh.indices.at(i));
				posIndices[1] = static_cast<int>(mcMesh.indices.at(i + 1u));
				posIndices[2] = static_cast<int>(mcMesh.indices.at(i + 2u));
				warpedMesh.addFace(&posIndices[0]);
			}
			warpedMesh.updateNormals(warpedVD, 0u);
			warpedMesh.updateVertexNormals(warpedVD);
			stats.meshConvertMs = elapsedMs(meshConvertStart);
			stats.vertexCount = nPoints;
			stats.faceCount = nFaces;
			stats.signature = computeSignature(mcMesh);
			stats.surfaceHash = stats.signature.hash;
			stats.success = true;
			stats.reconstructed = true;
			return stats;
		}

		template <typename ResolutionDerived>
		inline SDFSurfaceReconstructionStats reconstructIfDirty(
			bool& meshUpdated,
			SDFSurfaceSignature& currentSignature,
			std::vector<Real>& nodeSDFVals,
			const Eigen::MatrixBase<ResolutionDerived>& resolutionSDF,
			const Eigen::AlignedBox3d& gridDomain,
			MC::mcMesh& mcMesh,
			MC::MarchingCubeWorkspace& workspace,
			VertexData& warpedVD,
			Utilities::IndexedFaceMesh& warpedMesh)
		{
			if (!shouldReconstruct(meshUpdated, currentSignature))
			{
				SDFSurfaceReconstructionStats stats;
				stats.success = true;
				stats.signature = currentSignature;
				stats.vertexCount = currentSignature.vertexCount;
				stats.faceCount = currentSignature.faceCount;
				stats.surfaceHash = currentSignature.hash;
				return stats;
			}

			SDFSurfaceReconstructionStats stats = reconstruct(
				nodeSDFVals,
				resolutionSDF,
				gridDomain,
				mcMesh,
				workspace,
				warpedVD,
				warpedMesh);
			if (stats.success)
			{
				meshUpdated = false;
				currentSignature = stats.signature;
			}
			return stats;
		}
	}
}

#endif
