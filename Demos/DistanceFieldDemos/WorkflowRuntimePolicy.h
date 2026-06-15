#ifndef __WorkflowRuntimePolicy_h__
#define __WorkflowRuntimePolicy_h__

#include <cstdlib>
#include <string>
#include "Common/Common.h"

namespace PBD
{
	struct WorkflowRuntimePolicy
	{
		bool autoWorkflowBase = true;
		bool scriptedToolPlayback = false;
		bool autoStageAdvance = false;
		bool hapticToolControl = true;
		bool hapticAcceptSecondButton = false;
		bool hapticAutoCalibrateOffset = true;
		bool showParameterGui = true;
		bool liveHapticVisualContext = true;
		double hapticWorkspaceScale = 1.0;
		Vector3r hapticVisualOffset = Vector3r::Zero();
	};

	inline bool workflowRuntimeHasArg(const int argc, char** argv, const std::string& expected)
	{
		for (int i = 1; i < argc; i++)
		{
			if (std::string(argv[i]) == expected)
				return true;
		}
		return false;
	}

	inline double clampHapticWorkspaceScale(const double scale)
	{
		if (scale < 0.2)
			return 0.2;
		if (scale > 5.0)
			return 5.0;
		return scale;
	}

	inline double parseHapticWorkspaceScale(const int argc, char** argv)
	{
		for (int i = 1; i + 1 < argc; i++)
		{
			if (std::string(argv[i]) != "--haptic-workspace-scale")
				continue;
			char* end = nullptr;
			const double value = std::strtod(argv[i + 1], &end);
			if ((end != argv[i + 1]) && (value > 0.0))
				return clampHapticWorkspaceScale(value);
			return 1.0;
		}
		return 1.0;
	}

	inline bool parseWorkflowVector3(const std::string& text, Vector3r& out)
	{
		const size_t firstComma = text.find(',');
		const size_t secondComma = (firstComma == std::string::npos) ? std::string::npos : text.find(',', firstComma + 1u);
		if ((firstComma == std::string::npos) || (secondComma == std::string::npos))
			return false;
		const std::string xText = text.substr(0u, firstComma);
		const std::string yText = text.substr(firstComma + 1u, secondComma - firstComma - 1u);
		const std::string zText = text.substr(secondComma + 1u);
		char* endX = nullptr;
		char* endY = nullptr;
		char* endZ = nullptr;
		const double x = std::strtod(xText.c_str(), &endX);
		const double y = std::strtod(yText.c_str(), &endY);
		const double z = std::strtod(zText.c_str(), &endZ);
		if ((endX == nullptr) || (*endX != '\0') ||
			(endY == nullptr) || (*endY != '\0') ||
			(endZ == nullptr) || (*endZ != '\0'))
			return false;
		out = Vector3r(static_cast<Real>(x), static_cast<Real>(y), static_cast<Real>(z));
		return true;
	}

	inline Vector3r parseHapticVisualOffset(const int argc, char** argv)
	{
		for (int i = 1; i + 1 < argc; i++)
		{
			if (std::string(argv[i]) != "--haptic-visual-offset")
				continue;
			Vector3r offset = Vector3r::Zero();
			if (parseWorkflowVector3(argv[i + 1], offset))
				return offset;
			return Vector3r::Zero();
		}
		return Vector3r::Zero();
	}

	inline WorkflowRuntimePolicy parseWorkflowRuntimePolicy(const int argc, char** argv)
	{
		WorkflowRuntimePolicy policy;
		const bool autoDemo = workflowRuntimeHasArg(argc, argv, "--workflow-auto-demo");
		policy.autoWorkflowBase = true;
		policy.scriptedToolPlayback = autoDemo;
		policy.autoStageAdvance = autoDemo;
		policy.hapticToolControl = !autoDemo;
		policy.showParameterGui = false;
		policy.liveHapticVisualContext = !autoDemo;
		policy.hapticAcceptSecondButton = workflowRuntimeHasArg(argc, argv, "--haptic-two-buttons");
		policy.hapticAutoCalibrateOffset = !workflowRuntimeHasArg(argc, argv, "--haptic-no-auto-calibrate");
		policy.hapticWorkspaceScale = parseHapticWorkspaceScale(argc, argv);
		policy.hapticVisualOffset = parseHapticVisualOffset(argc, argv);
		return policy;
	}
}

#endif
