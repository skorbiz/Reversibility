/*
 * ErrorCheck.cpp
 *
 *  Created on: Jan 9, 2015
 *      Author: josl
 */

#include "ErrorCheck.hpp"

namespace dsl {

ErrorCheck::ErrorCheck(dsl::Identification id, dsl::Condition * condition, dsl::MessagesError * msgError, bool isReversible, bool isSymmetric, bool isSwapped, bool isUncallLayout) :
		Instruction(id, isReversible, isSymmetric, isSwapped, isUncallLayout),
		_condition(condition),
		_msgError(msgError)
{
}

ErrorCheck::~ErrorCheck()
{
}

dsl::Condition * ErrorCheck::getCondition()
{
	return _condition;
}

void ErrorCheck::setCondition(dsl::Condition * condition)
{
	_condition = condition;
}

// **********************************
// *** EXECUTION FUNCTION

void ErrorCheck::doExecuteForward()
{
	std::cout << "Executed ErrorCheck Instruction";
	if(_condition->getEvaluation() == false)
	{
		_msgError->increaseErrorCount();
		postMessageToController(_msgError);
		std::cout << " :: check failed :: posted error messages" << std::endl;

	}
	else
	{
		_msgError->resetErrorCount();
		std::cout << " :: check passed" << std::endl;
	}
}

void ErrorCheck::doExecuteBackward()
{
	std::cout << " Skipped error check in reverse execution" << std::endl;
}

std::string ErrorCheck::getType()
{
	return "error check";
}

void ErrorCheck::stateUpdate(dsl::State & state)
{
	_condition->stateUpdate(state);
}


} /* namespace dsl */
