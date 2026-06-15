#ifndef __LiveHapticToolControl_h__
#define __LiveHapticToolControl_h__

#include "Common/Common.h"

namespace PBD
{
	namespace DemoHaptics
	{
		struct LiveHapticToolSample
		{
			LiveHapticToolSample();
			LiveHapticToolSample(const bool activeValue, const Vector3r& positionValue);

			bool active;
			Vector3r position;
		};

		inline LiveHapticToolSample::LiveHapticToolSample() :
			active(false),
			position(Vector3r::Zero())
		{
		}

		inline LiveHapticToolSample::LiveHapticToolSample(const bool activeValue, const Vector3r& positionValue) :
			active(activeValue),
			position(positionValue)
		{
		}

		inline LiveHapticToolSample makeLiveHapticToolSample(const bool active, const Vector3r& position)
		{
			return LiveHapticToolSample(active, position);
		}

		inline bool liveHapticSelectionStateFromButtons(const bool firstButtonDown, const bool secondButtonDown)
		{
			return firstButtonDown || secondButtonDown;
		}

		inline LiveHapticToolSample offsetLiveHapticToolSample(
			const LiveHapticToolSample& sample,
			const Vector3r& offset)
		{
			return LiveHapticToolSample(sample.active, sample.position + offset);
		}

		inline Vector3r scaleLiveHapticProxyPosition(
			const Vector3r& proxyPosition,
			const double workspaceScale)
		{
			return proxyPosition * static_cast<Real>(workspaceScale);
		}

		template <typename TransformScalar>
		inline Vector3r liveHapticProxyTransformTranslation(const TransformScalar* const proxyTransform)
		{
			return Vector3r(
				static_cast<Real>(proxyTransform[12]),
				static_cast<Real>(proxyTransform[13]),
				static_cast<Real>(proxyTransform[14]));
		}

		template <typename TransformScalar>
		inline void scaleLiveHapticProxyTransformTranslation(
			TransformScalar* const proxyTransform,
			const double workspaceScale)
		{
			const Vector3r scaledPosition = scaleLiveHapticProxyPosition(
				liveHapticProxyTransformTranslation(proxyTransform),
				workspaceScale);
			proxyTransform[12] = static_cast<TransformScalar>(scaledPosition.x());
			proxyTransform[13] = static_cast<TransformScalar>(scaledPosition.y());
			proxyTransform[14] = static_cast<TransformScalar>(scaledPosition.z());
		}

		inline bool hasActiveLiveHapticTool(const LiveHapticToolSample& sample)
		{
			return sample.active;
		}

		inline bool liveHapticToolVisualUsesProxyTransform(const bool hapticAvailable)
		{
			(void)hapticAvailable;
			return false;
		}

		inline bool liveHapticToolSampleShouldRelocateMesh(const bool hapticAvailable)
		{
			(void)hapticAvailable;
			return true;
		}

		inline Real liveHapticToolVisibilityMarkerRadius(const Real toolRadius)
		{
			const Real scaledRadius = toolRadius * static_cast<Real>(1.8);
			const Real minimumRadius = static_cast<Real>(0.35);
			return (scaledRadius > minimumRadius) ? scaledRadius : minimumRadius;
		}

		inline Real liveHapticSDFEditRadius(const Real toolRadius)
		{
			const Real scaledRadius = toolRadius * static_cast<Real>(1.75);
			const Real minimumRadius = static_cast<Real>(0.60);
			return (scaledRadius > minimumRadius) ? scaledRadius : minimumRadius;
		}

		inline Real liveHapticSoftTissueFractureRadius(const Real toolRadius)
		{
			const Real scaledRadius = toolRadius * static_cast<Real>(3.5);
			const Real minimumRadius = static_cast<Real>(0.60);
			return (scaledRadius > minimumRadius) ? scaledRadius : minimumRadius;
		}

		template <typename ProbeLike>
		void deactivateProbeIfInactive(const LiveHapticToolSample& sample, ProbeLike& probe)
		{
			if (!sample.active)
				probe.active = false;
		}
	}
}

#endif
