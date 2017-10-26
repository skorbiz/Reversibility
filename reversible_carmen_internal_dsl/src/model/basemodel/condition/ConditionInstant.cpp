/*
 * ConditionInstant.cpp
 *
 *  Created on: Jan 8, 2015
 *      Author: josl
 */

#include "ConditionInstant.hpp"

namespace dsl
{

ConditionInstant::ConditionInstant() :
	_hasBeenEvaluated(false),
	_result(false)
{
}

ConditionInstant::~ConditionInstant()
{
}


// **********************************
// *** Overwritten functions
bool ConditionInstant::getEvaluation()
{
	_result = evaluate();
	_hasBeenEvaluated = true;
	return _result;
}

// **********************************
// *** virtual functions
bool ConditionInstant::evaluate()
{
	std::cerr << "Error, condition had not overwritten the evaluate functions" << std::endl;
	exit(-1);
	return false;
}

//

} /* namespace dsl */
