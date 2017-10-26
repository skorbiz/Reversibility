/*
 * Assignment.cpp
 *
 *  Created on: Jun 20, 2017
 *      Author: josl
 */

#include "Assignment.h"

#include <iostream>

#include "../MemoryModel.hpp"
#include "../Variable.h"
#include "../Variables.h"

namespace model {

Assignment::Assignment(std::string text, std::string variable) :
		_textToAssign(text),
		_variable(variable)
{
}

Assignment::~Assignment()
{
}

ExecutionResult23 Assignment::execute()
{
	if(variables->varString.doContain(_variable))
	{
		variables->varString.getRef(_variable).set(_textToAssign);
		//std::cout << __PRETTY_FUNCTION__ << " " << _variable << " = " << _textToAssign << std::endl;
	}
	else
		std::cout << "Warning: Assignment::execute() found no variable with name " << _variable << std::endl;


	//std::shared_ptr<std::string> text = std::make_shared<std::string>(_textToAssign);
	//_variable.set(_textToAssign);
	ExecutionResult23 r;
	return r;
}

ExecutionResult23 Assignment::executeBackwards()
{
	//_variable.set(_textToAssign);
	ExecutionResult23 r;
	return r;
}


} /* namespace model */
