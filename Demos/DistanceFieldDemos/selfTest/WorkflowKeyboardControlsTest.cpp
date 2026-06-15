#include "Demos/DistanceFieldDemos/WorkflowKeyboardControls.h"

#include <cstdlib>
#include <iostream>

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
}

int main()
{
	using PBD::WorkflowKeyboardCommand;
	using PBD::workflowKeyboardCommandFromGlfwKey;

	require(workflowKeyboardCommandFromGlfwKey(GLFW_KEY_N, GLFW_PRESS) == WorkflowKeyboardCommand::NextStage,
		"physical N press must advance the workflow even when text input is unavailable.");
	require(workflowKeyboardCommandFromGlfwKey(GLFW_KEY_N, GLFW_REPEAT) == WorkflowKeyboardCommand::NextStage,
		"physical N repeat must advance the workflow.");
	require(workflowKeyboardCommandFromGlfwKey(GLFW_KEY_N, GLFW_RELEASE) == WorkflowKeyboardCommand::None,
		"physical N release must not advance the workflow.");
	require(workflowKeyboardCommandFromGlfwKey(GLFW_KEY_R, GLFW_PRESS) == WorkflowKeyboardCommand::Reset,
		"physical R press must reset the workflow.");
	require(workflowKeyboardCommandFromGlfwKey(GLFW_KEY_1, GLFW_PRESS) == WorkflowKeyboardCommand::Stage1,
		"physical 1 press must jump to stage 1.");
	require(workflowKeyboardCommandFromGlfwKey(GLFW_KEY_2, GLFW_PRESS) == WorkflowKeyboardCommand::Stage2,
		"physical 2 press must jump to stage 2.");
	require(workflowKeyboardCommandFromGlfwKey(GLFW_KEY_3, GLFW_PRESS) == WorkflowKeyboardCommand::Stage3,
		"physical 3 press must jump to stage 3.");
	require(workflowKeyboardCommandFromGlfwKey(GLFW_KEY_A, GLFW_PRESS) == WorkflowKeyboardCommand::None,
		"unmapped physical keys must not trigger workflow commands.");

	std::cout << "WorkflowKeyboardControlsTest ok\n";
	return 0;
}
