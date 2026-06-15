#include "Demos/DistanceFieldDemos/WorkflowRuntimePolicy.h"

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

PBD::WorkflowRuntimePolicy parse(std::initializer_list<const char*> args)
{
	std::vector<char*> argv;
	argv.reserve(args.size());
	for (const char* arg : args)
		argv.push_back(const_cast<char*>(arg));
	return PBD::parseWorkflowRuntimePolicy(static_cast<int>(argv.size()), argv.data());
}
}

int main()
{
	{
		const PBD::WorkflowRuntimePolicy policy = parse({ "SpineWorkflowDemo" });
		require(policy.autoWorkflowBase, "no-argument startup must use the auto workflow stage/result pipeline.");
		require(policy.hapticToolControl, "no-argument startup must use haptic tool control.");
		require(!policy.scriptedToolPlayback, "no-argument startup must not use scripted code-move tool playback.");
		require(!policy.autoStageAdvance, "no-argument startup must not auto-advance stages.");
		require(!policy.showParameterGui, "no-argument haptic workflow must keep the workflow parameter panel hidden.");
		require(policy.liveHapticVisualContext, "no-argument haptic workflow must use the independent-demo visual path.");
		require(!policy.hapticAcceptSecondButton, "default haptic workflow must use TouchX-compatible single-button mode.");
		require(policy.hapticAutoCalibrateOffset, "default haptic workflow must auto-calibrate the first live press.");
		require(policy.hapticWorkspaceScale == 1.0, "default haptic workspace scale must preserve the independent demo mapping.");
		require(policy.hapticVisualOffset == Vector3r::Zero(), "default haptic visual offset must not move the haptic workspace.");
	}
	{
		const PBD::WorkflowRuntimePolicy policy = parse({ "SpineWorkflowDemo", "--haptic-two-buttons" });
		require(policy.hapticAcceptSecondButton, "--haptic-two-buttons must enable the second haptic button.");
	}
	{
		const PBD::WorkflowRuntimePolicy policy = parse({ "SpineWorkflowDemo", "--haptic-no-auto-calibrate" });
		require(!policy.hapticAutoCalibrateOffset, "--haptic-no-auto-calibrate must disable first-press calibration.");
	}
	{
		const PBD::WorkflowRuntimePolicy policy = parse({ "SpineWorkflowDemo", "--workflow-auto-demo" });
		require(policy.autoWorkflowBase, "--workflow-auto-demo must use the auto workflow pipeline.");
		require(policy.scriptedToolPlayback, "--workflow-auto-demo must use scripted code-move tool playback.");
		require(policy.autoStageAdvance, "--workflow-auto-demo must auto-advance stages.");
		require(!policy.hapticToolControl, "auto workflow must not also consume haptic tool control.");
		require(!policy.showParameterGui, "auto workflow must keep the operation panel hidden.");
		require(!policy.liveHapticVisualContext, "auto workflow must keep the performance visual path.");
	}
	{
		const PBD::WorkflowRuntimePolicy policy = parse({ "SpineWorkflowDemo", "--workflow-manual-demo" });
		require(policy.autoWorkflowBase, "--workflow-manual-demo must keep the auto workflow pipeline.");
		require(policy.hapticToolControl, "--workflow-manual-demo must explicitly select haptic tool control.");
		require(!policy.scriptedToolPlayback, "--workflow-manual-demo must not use scripted code-move playback.");
		require(!policy.autoStageAdvance, "--workflow-manual-demo must not auto-advance stages.");
		require(policy.liveHapticVisualContext, "--workflow-manual-demo must use the independent-demo visual path.");
	}
	{
		const PBD::WorkflowRuntimePolicy policy = parse({ "SpineWorkflowDemo", "--haptic-workspace-scale", "1.6" });
		require(policy.hapticWorkspaceScale > 1.59 && policy.hapticWorkspaceScale < 1.61,
			"--haptic-workspace-scale must configure the live haptic mapping scale.");
	}
	{
		const PBD::WorkflowRuntimePolicy policy = parse({ "SpineWorkflowDemo", "--haptic-workspace-scale", "0.01" });
		require(policy.hapticWorkspaceScale == 0.2,
			"--haptic-workspace-scale must clamp very small values to a usable minimum.");
	}
	{
		const PBD::WorkflowRuntimePolicy policy = parse({ "SpineWorkflowDemo", "--haptic-visual-offset", "1.5,-2.0,0.25" });
		require((policy.hapticVisualOffset - Vector3r(1.5, -2.0, 0.25)).norm() < 1e-6,
			"--haptic-visual-offset must parse a comma-separated xyz offset.");
	}

	std::cout << "WorkflowRuntimePolicyTest ok\n";
	return 0;
}
