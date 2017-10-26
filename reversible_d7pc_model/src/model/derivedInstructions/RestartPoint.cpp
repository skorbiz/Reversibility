/*
 * RestartPoint.cpp
 *
 *  Created on: Jun 22, 2017
 *      Author: josl
 */

#include <model/derivedInstructions/RestartPoint.h>
#include <iostream>

namespace model {

RestartPoint::RestartPoint()
{
}

RestartPoint::~RestartPoint()
{
}

ExecutionResult23 RestartPoint::execute()
{
	std::cout << __PRETTY_FUNCTION__ << std::endl;
	ExecutionResult23 r;
	return r;
}

ExecutionResult23 RestartPoint::executeBackwards()
{
	std::cout << __PRETTY_FUNCTION__ << std::endl;
	ExecutionResult23 r;
	return r;
}


} /* namespace model */
