/*
 * MessagesError.cpp
 *
 *  Created on: Jan 9, 2015
 *      Author: josl
 */

#include "MessagesError.hpp"

namespace dsl {

MessagesError::MessagesError() :
		_errorCount(0)
{
}

MessagesError::~MessagesError()
{
}

int MessagesError::getErrorCount()
{
	return _errorCount;
}

void MessagesError::resetErrorCount()
{
	_errorCount = 0;
}

void MessagesError::increaseErrorCount()
{
	_errorCount++;
}


} /* namespace dsl */
