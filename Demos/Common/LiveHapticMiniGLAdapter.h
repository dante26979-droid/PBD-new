#ifndef __LiveHapticMiniGLAdapter_h__
#define __LiveHapticMiniGLAdapter_h__

#include "Demos/Common/LiveHapticToolControl.h"
#include "Demos/Visualization/MiniGL.h"

namespace PBD
{
	namespace DemoHaptics
	{
		inline LiveHapticToolSample sampleMiniGLLiveHapticTool()
		{
			if (!MiniGL::isHapticAvailable())
				return makeLiveHapticToolSample(false, Vector3r::Zero());
			MiniGL::refreshHapticButtonState();
			return makeLiveHapticToolSample(MiniGL::getHapticSelectionState(), MiniGL::getHapticPos());
		}
	}
}

#endif
