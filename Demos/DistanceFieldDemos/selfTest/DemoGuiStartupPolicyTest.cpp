#include "Demos/Common/DemoGuiStartupPolicy.h"

#include <cstdlib>
#include <initializer_list>
#include <iostream>
#include <vector>

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

PBD::DemoGuiStartupPolicy parse(std::initializer_list<const char*> args)
{
	std::vector<char*> argv;
	argv.reserve(args.size());
	for (const char* arg : args)
		argv.push_back(const_cast<char*>(arg));
	return PBD::parseDemoGuiStartupPolicy(static_cast<int>(argv.size()), argv.data());
}
}

int main()
{
	{
		const auto policy = parse({ "FractureDemo" });
		require(policy.initImgui, "default startup must still initialize ImGui input/backend.");
		require(!policy.showPanel, "default startup must hide the GUI panel.");
	}
	{
		const auto policy = parse({ "FractureDemo", "--show-gui" });
		require(policy.initImgui, "--show-gui must keep ImGui initialized.");
		require(policy.showPanel, "--show-gui must make the GUI panel visible on startup.");
	}
	{
		const auto policy = parse({ "FractureDemo", "--show-gui", "--hide-gui" });
		require(!policy.showPanel, "--hide-gui must override an earlier --show-gui.");
	}
	{
		const auto policy = parse({ "FractureDemo", "--tool-cut-minigl-screenshot-smoke", "--show-gui" });
		require(!policy.initImgui, "screenshot smoke must skip ImGui even when --show-gui is present.");
	}
	require(PBD::isDemoBaseArgumentWithValue("--record-hit-path"),
		"DemoBase must recognize --record-hit-path as an option with a value, not a scene file.");
	require(PBD::isDemoBaseArgumentWithValue("--haptic-workspace-scale"),
		"DemoBase must recognize --haptic-workspace-scale as an option with a value, not a scene file.");
	require(PBD::isDemoBaseArgumentWithValue("--haptic-visual-offset"),
		"DemoBase must recognize --haptic-visual-offset as an option with a value, not a scene file.");
	require(PBD::isDemoBaseFlagArgument("--workflow-auto-demo"),
		"DemoBase must recognize --workflow-auto-demo as a flag, not a scene file.");
	require(PBD::isDemoBaseFlagArgument("--workflow-manual-demo"),
		"DemoBase must recognize --workflow-manual-demo as a flag, not a scene file.");
	require(PBD::isDemoBaseFlagArgument("--workflow-haptic-diagnostics"),
		"DemoBase must recognize --workflow-haptic-diagnostics as a flag, not a scene file.");
	require(PBD::isDemoBaseFlagArgument("--haptic-init-smoke"),
		"DemoBase must recognize --haptic-init-smoke as a flag, not a scene file.");
	std::cout << "demo_gui_startup_policy_status=ok\n";
	return 0;
}
