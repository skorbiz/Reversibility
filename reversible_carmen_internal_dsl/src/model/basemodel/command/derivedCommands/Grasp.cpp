/*
-std=c++11 * Grasp.cpp
 *
 *  Created on: Jan 20, 2015
 *      Author: josl
 */

#include "Grasp.hpp"

namespace dsl {

Grasp::Grasp(std::shared_ptr<RobotiqInterfaceProxy> RIQP, int q, int speed, int force) :
		_RIQP(RIQP),
		_qTo(q),
		_qFrom(-1),
		_speed(speed),
		_force(force)
{
}

Grasp::~Grasp()
{
}

void Grasp::execute()
{
//	gen::dCommand << "executed Grasp" << std::endl;
	_RIQP->setForce(_force);
	_RIQP->setSpeed(_speed);
	_RIQP->setQ(_qTo);
}

void Grasp::executeBackwards()
{
//	gen::dCommand << "executed Grasp backwards" << std::endl;
	_RIQP->setForce(_force);
	_RIQP->setSpeed(_speed);
	_RIQP->setQ(_qFrom);
}

std::string Grasp::getType()
{
	return "grasp";
}

void Grasp::stateUpdate(dsl::State & state)
{
	if(state.getGripper() > -1)
	_qFrom = state.getGripper();
	state.updateGripper(_qTo);
}

void Grasp::stateUpdateSwapped(dsl::State & state)
{
	state.updateGripper(_qFrom);
}

std::string Grasp::getGraspStatus(int q) const
{
	if(q == 0)
		return "open";
	if(q == 255)
		return "closed";
	return "val" + std::to_string(q);

}

std::string Grasp::getArgumentForward() const
{
	return getGraspStatus(_qTo);
}

std::string Grasp::getArgumentBackward() const
{
	return getGraspStatus(_qFrom);
}




} /* namespace dsl */


