/*
 * AssemblyProgram.cpp
 *
 *  Created on: Nov 17, 2014
 *      Author: josl
 */

#include "AssemblyProgram.hpp"

namespace dsl {

AssemblyProgram::AssemblyProgram()
{
}

AssemblyProgram::~AssemblyProgram()
{
}

dsl::BaseProgram AssemblyProgram::getProgram()
{
	buildExpandedModel();
	return getFinalProgram();
}

} /* namespace dsl */
