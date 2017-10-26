/*
 * Condition.cpp
 *
 *  Created on: Jan 8, 2015
 *      Author: josl
 */

#include "Condition.hpp"

namespace dsl {

Condition::Condition()
{
}

Condition::~Condition()
{
}

void Condition::stateUpdate(dsl::State const& state)
{
	std::cout << "Test of implicit state update in conditions" << std::endl;
}


} /* namespace dsl */
