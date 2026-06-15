#ifndef __WorkflowKeyboardControls_h__
#define __WorkflowKeyboardControls_h__

#include "GLFW/glfw3.h"

namespace PBD
{
	enum class WorkflowKeyboardCommand
	{
		None = 0,
		Reset,
		NextStage,
		Stage1,
		Stage2,
		Stage3
	};

	inline bool workflowKeyboardActionTriggersCommand(const int action)
	{
		return (action == GLFW_PRESS) || (action == GLFW_REPEAT);
	}

	inline WorkflowKeyboardCommand workflowKeyboardCommandFromGlfwKey(const int key, const int action)
	{
		if (!workflowKeyboardActionTriggersCommand(action))
			return WorkflowKeyboardCommand::None;

		switch (key)
		{
		case GLFW_KEY_R:
			return WorkflowKeyboardCommand::Reset;
		case GLFW_KEY_N:
			return WorkflowKeyboardCommand::NextStage;
		case GLFW_KEY_1:
			return WorkflowKeyboardCommand::Stage1;
		case GLFW_KEY_2:
			return WorkflowKeyboardCommand::Stage2;
		case GLFW_KEY_3:
			return WorkflowKeyboardCommand::Stage3;
		default:
			return WorkflowKeyboardCommand::None;
		}
	}
}

#endif
