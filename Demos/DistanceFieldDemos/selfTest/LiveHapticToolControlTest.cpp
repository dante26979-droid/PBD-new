#include "Demos/Common/LiveHapticToolControl.h"

#include <iostream>

namespace
{
	struct FakeProbe
	{
		FakeProbe() :
			active(true)
		{
		}

		bool active;
	};

	bool expectTrue(const bool condition, const char* label)
	{
		if (condition)
			return true;
		std::cerr << label << " failed\n";
		return false;
	}
}

int main()
{
	bool ok = true;

	const Vector3r livePosition(static_cast<Real>(1.0), static_cast<Real>(2.0), static_cast<Real>(3.0));
	const PBD::DemoHaptics::LiveHapticToolSample activeSample =
		PBD::DemoHaptics::makeLiveHapticToolSample(true, livePosition);
	ok &= expectTrue(PBD::DemoHaptics::hasActiveLiveHapticTool(activeSample), "active sample state");
	ok &= expectTrue((activeSample.position - livePosition).norm() < static_cast<Real>(1e-9), "active sample position");
	ok &= expectTrue(
		PBD::DemoHaptics::liveHapticSelectionStateFromButtons(true, false),
		"first haptic button activates live tool");
	ok &= expectTrue(
		PBD::DemoHaptics::liveHapticSelectionStateFromButtons(false, true),
		"second haptic button activates live tool");
	ok &= expectTrue(
		!PBD::DemoHaptics::liveHapticSelectionStateFromButtons(false, false),
		"released haptic buttons deactivate live tool");
	const Vector3r viewOffset(static_cast<Real>(0.5), static_cast<Real>(-1.0), static_cast<Real>(2.0));
	const PBD::DemoHaptics::LiveHapticToolSample offsetSample =
		PBD::DemoHaptics::offsetLiveHapticToolSample(activeSample, viewOffset);
	ok &= expectTrue(PBD::DemoHaptics::hasActiveLiveHapticTool(offsetSample), "offset sample preserves active state");
	ok &= expectTrue(
		(offsetSample.position - (livePosition + viewOffset)).norm() < static_cast<Real>(1e-9),
		"offset sample position");

	const PBD::DemoHaptics::LiveHapticToolSample inactiveSample =
		PBD::DemoHaptics::makeLiveHapticToolSample(false, livePosition);
	ok &= expectTrue(!PBD::DemoHaptics::hasActiveLiveHapticTool(inactiveSample), "inactive sample state");
	ok &= expectTrue((inactiveSample.position - livePosition).norm() < static_cast<Real>(1e-9), "inactive sample tracks position");

	FakeProbe probe;
	PBD::DemoHaptics::deactivateProbeIfInactive(inactiveSample, probe);
	ok &= expectTrue(!probe.active, "inactive probe reset");
	ok &= expectTrue(
		!PBD::DemoHaptics::liveHapticToolVisualUsesProxyTransform(true),
		"haptic visual should use relocated mesh when haptic is available");
	ok &= expectTrue(
		!PBD::DemoHaptics::liveHapticToolVisualUsesProxyTransform(false),
		"haptic visual should not use proxy transform when haptic is unavailable");
	ok &= expectTrue(
		PBD::DemoHaptics::liveHapticToolSampleShouldRelocateMesh(true),
		"haptic visual should relocate mesh vertices when haptic is available");
	ok &= expectTrue(
		PBD::DemoHaptics::liveHapticToolSampleShouldRelocateMesh(false),
		"haptic visual should relocate mesh vertices for non-haptic fallback");
	ok &= expectTrue(
		PBD::DemoHaptics::liveHapticToolVisibilityMarkerRadius(static_cast<Real>(0.10)) >= static_cast<Real>(0.35),
		"haptic visual marker should remain visible for small tools");
	ok &= expectTrue(
		PBD::DemoHaptics::liveHapticToolVisibilityMarkerRadius(static_cast<Real>(0.50)) > static_cast<Real>(0.50),
		"haptic visual marker should scale with large tools");
	ok &= expectTrue(
		PBD::DemoHaptics::liveHapticSDFEditRadius(static_cast<Real>(0.35)) >= static_cast<Real>(0.60),
		"live haptic SDF edit radius should tolerate visual/SDF contact offsets");
	ok &= expectTrue(
		PBD::DemoHaptics::liveHapticSoftTissueFractureRadius(static_cast<Real>(0.10)) >=
		PBD::DemoHaptics::liveHapticToolVisibilityMarkerRadius(static_cast<Real>(0.10)),
		"live haptic soft tissue fracture radius should cover the visible control marker");
	ok &= expectTrue(
		PBD::DemoHaptics::liveHapticSoftTissueFractureRadius(static_cast<Real>(0.10)) >= static_cast<Real>(0.60),
		"live haptic soft tissue fracture radius should tolerate surface-to-tet-edge visual offsets");
	ok &= expectTrue(
		PBD::DemoHaptics::liveHapticSoftTissueFractureRadius(static_cast<Real>(0.10)) > static_cast<Real>(0.10),
		"live haptic soft tissue fracture radius should exceed the tiny model tool radius");
	const Vector3r rawProxyPosition(static_cast<Real>(1.0), static_cast<Real>(2.0), static_cast<Real>(-3.0));
	const Vector3r scaledProxyPosition =
		PBD::DemoHaptics::scaleLiveHapticProxyPosition(rawProxyPosition, 1.6);
	ok &= expectTrue(
		(scaledProxyPosition - Vector3r(static_cast<Real>(1.6), static_cast<Real>(3.2), static_cast<Real>(-4.8))).norm() <
		static_cast<Real>(1e-9),
		"haptic proxy position should use workspace scale before workflow sampling");
	double proxyTransform[16] = {
		1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0,
		1.0, 2.0, -3.0, 1.0
	};
	PBD::DemoHaptics::scaleLiveHapticProxyTransformTranslation(proxyTransform, 1.6);
	ok &= expectTrue(
		std::abs(proxyTransform[12] - 1.6) < 1e-9 &&
		std::abs(proxyTransform[13] - 3.2) < 1e-9 &&
		std::abs(proxyTransform[14] + 4.8) < 1e-9,
		"haptic proxy transform translation should match scaled sample position");

	if (!ok)
		return 1;

	std::cout << "LiveHapticToolControlTest ok\n";
	return 0;
}
