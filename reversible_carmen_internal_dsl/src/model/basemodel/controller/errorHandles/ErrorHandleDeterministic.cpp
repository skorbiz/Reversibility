/*
 * ErrorHandleDeterministic.cpp
 *
 *  Created on: Apr 8, 2015
 *      Author: josl
 */

#include "ErrorHandleDeterministic.hpp"

namespace dsl {

ErrorHandleDeterministic::ErrorHandleDeterministic() :
		_inErrorMode(false),
		_numberOfSequencesLeftToHandleError(0)
{
}

ErrorHandleDeterministic::~ErrorHandleDeterministic()
{
}

void ErrorHandleDeterministic::update(dsl::MessagesError & msg)
{
	_inErrorMode = true;
	_numberOfSequencesLeftToHandleError = 4 * msg.getErrorCount();
	cerr << "new error - deterministic handling : ";
}

void ErrorHandleDeterministic::update(std::shared_ptr<dsl::Instruction> ins)
{
	if(_inErrorMode == false)
		return;

	std::cout << "INS DPETH::::: " << ins->getIDClass().getDepth() << std::endl;

	if(ins->getIDClass().getDepth() < 3)
	if(ins->getType().compare("empty linker") == false)
		_numberOfSequencesLeftToHandleError--;

	if(_numberOfSequencesLeftToHandleError < 1)
		_inErrorMode = false;

	cerr << " steps left = " << _numberOfSequencesLeftToHandleError << std::endl;
}

bool ErrorHandleDeterministic::isInErrorMode()
{
	return _inErrorMode;
}

} /* namespace dsl */
