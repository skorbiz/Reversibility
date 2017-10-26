/*
 * ErrorHandleRandom.cpp
 *
 *  Created on: Apr 8, 2015
 *      Author: josl
 */

#include "ErrorHandleRandom.hpp"

namespace dsl {

ErrorHandleRandom::ErrorHandleRandom() :
		_inErrorMode(false),
		_numberOfStepsLeftToHandleError(0)
{
}

ErrorHandleRandom::~ErrorHandleRandom() {
}


void ErrorHandleRandom::update(dsl::MessagesError & msg)
{
	_inErrorMode = true;

    std::mt19937 rng;
    rng.seed(std::random_device()());										//Todo dont have random state in program
    std::uniform_int_distribution<std::mt19937::result_type> dist(0,10); 	// distribution in range [1, 6]

	int nOffsetSteps = 10 * msg.getErrorCount();
	int nRandomSteps = dist(rng);
	_numberOfStepsLeftToHandleError = nOffsetSteps + nRandomSteps;
	cerr << "new error : ";
}

void ErrorHandleRandom::update(std::shared_ptr<dsl::Instruction> ins)
{
	if(_inErrorMode == false)
		return;

	_numberOfStepsLeftToHandleError--;

	if(_numberOfStepsLeftToHandleError < 1)
		_inErrorMode = false;

	cerr << "steps left = " << _numberOfStepsLeftToHandleError << std::endl;
}

bool ErrorHandleRandom::isInErrorMode()
{
	return _inErrorMode;
}

} /* namespace dsl */
