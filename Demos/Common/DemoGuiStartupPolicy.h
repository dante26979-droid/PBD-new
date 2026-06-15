#ifndef __DemoGuiStartupPolicy_h__
#define __DemoGuiStartupPolicy_h__

#include <string>

namespace PBD
{
	struct DemoGuiStartupPolicy
	{
		bool initImgui = true;
		bool showPanel = false;
	};

	inline bool isDemoBaseArgumentWithValue(const std::string& arg)
	{
		return (arg == "--tool-cut-minigl-screenshot-smoke-frames") ||
			(arg == "--record-hit-path") ||
			(arg == "--steps-per-render") ||
			(arg == "--haptic-workspace-scale") ||
			(arg == "--haptic-visual-offset");
	}

	inline bool isDemoBaseFlagArgument(const std::string& arg)
	{
		return (arg == "--demo-flow-summary") ||
			(arg == "--show-gui") ||
			(arg == "--hide-gui") ||
			(arg == "--tool-cut-minigl-screenshot-smoke") ||
			(arg == "--workflow-minigl-screenshot-smoke") ||
			(arg == "--workflow-manual-demo") ||
			(arg == "--workflow-auto-demo") ||
			(arg == "--workflow-auto-demo-smoke") ||
			(arg == "--workflow-haptic-diagnostics") ||
			(arg == "--haptic-init-smoke");
	}

	inline DemoGuiStartupPolicy parseDemoGuiStartupPolicy(const int argc, char** argv)
	{
		DemoGuiStartupPolicy policy;
		for (int i = 1; i < argc; i++)
		{
			const std::string arg = argv[i];
			if ((arg == "--tool-cut-minigl-screenshot-smoke") ||
				(arg == "--workflow-minigl-screenshot-smoke"))
			{
				policy.initImgui = false;
			}
			else if (arg == "--show-gui")
			{
				policy.showPanel = true;
			}
			else if (arg == "--hide-gui")
			{
				policy.showPanel = false;
			}

			if (isDemoBaseArgumentWithValue(arg) && (i + 1 < argc))
				i++;
		}
		if (!policy.initImgui)
			policy.showPanel = false;
		return policy;
	}
}

#endif
