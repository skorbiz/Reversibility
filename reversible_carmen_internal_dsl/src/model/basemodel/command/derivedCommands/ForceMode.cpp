/*
 * ForceMode.cpp
 *
 *  Created on: Jan 20, 2015
 *      Author: josl
 */

#include "ForceMode.hpp"

namespace dsl {

ForceMode::ForceMode(std::shared_ptr<rwhw::UniversalRobotsRTLogging> urrt, std::shared_ptr<rwhw::URCallBackInterface> ur, ForceModeArgument fma_forward, ForceModeArgument fma_backward) :
		_urrt(urrt),
		_ur(ur),
		_fma_forward(fma_forward),
		_fma_backward(fma_backward)
{
	std::cerr << "ForceMode still needs to be updated to new interface" << std::endl;
	exit(-1);
}

ForceMode::~ForceMode()
{
}

void ForceMode::execute()
{
//	gen::dCommand << "executed ForceMode :: ";
	executeFMA(_fma_forward);
}

void ForceMode::executeBackwards()
{
//	gen::dCommand << "Reverse executed ForceMode :: ";
	executeFMA(_fma_backward);
}

void ForceMode::executeFMA(ForceModeArgument fma)
{

	if(fma.getActivationSignal())
	{
//		gen::dCommand << "activate" << std::endl;
//		_ur->forceModeStart(
//			fma.getType(),
//			fma.getTaskFrame(),
//			fma.getSelectionVector(),
//			fma.getWrench(),
//			fma.getLimits());
	}
	else
	{
//		gen::dCommand << "deativate" << std::endl;
		_ur->forceModeEnd();
	}
}

void ForceMode::stateUpdate(dsl::State & state)
{
	rw::math::Q q;
	state.update(q);
}

void ForceMode::stateUpdateSwapped(dsl::State & state)
{
	rw::math::Q q;
	state.update(q);
}


std::string ForceMode::getType()
{
	return "force mode";
}

} /* namespace dsl */
