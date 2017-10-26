/*
 * Nothing.cpp
 *
 *  Created on: Jun 22, 2017
 *      Author: josl
 */

#include <model/derivedInstructions/Nothing.h>
#include <iostream>

namespace model {
class ExecutionResult23;
} /* namespace model */

namespace model {

Nothing::Nothing()
{
}

Nothing::~Nothing()
{
}

ExecutionResult23 Nothing::execute()
{
	std::cout << __PRETTY_FUNCTION__ << std::endl;
	ExecutionResult23 r;
	return r;
}
ExecutionResult23 Nothing::executeBackwards()
{
	std::cout << __PRETTY_FUNCTION__ << std::endl;
	ExecutionResult23 r;
	return r;
}


} /* namespace model */
