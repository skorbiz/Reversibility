/*
 * ElementErrorCheck.cpp
 *
 *  Created on: Jan 13, 2015
 *      Author: josl
 */

#include "ElementErrorCheck.hpp"

namespace dsl
{

ElementErrorCheck::ElementErrorCheck(dsl::Condition * condition) :
		_insErrorCheck(nullptr),
		_condition(condition),
		_msgError(nullptr)
{
}

ElementErrorCheck::~ElementErrorCheck()
{
}

void ElementErrorCheck::buildIntermediateModel(direction dir, bool isUncallLayout, bool isReversible, dsl::Identification::Ptr id, dsl::Identification::Ptr next, dsl::Identification::Ptr previous, MemoryModel<dsl::IntermediateModel*> & memory)
{
	bool switchedDirection = false;
	if(dir == direction::REVERSE)
		switchedDirection = true;

	bool isReversibleLocal = isReversible and _isReversible;
	bool isSymmetric = false;
	bool isSwapped = switchedDirection;
	bool issUncallLayout = isUncallLayout;

	_msgError = new dsl::MessagesError();
	_insErrorCheck = std::make_shared<dsl::ErrorCheck>(*id, _condition, _msgError, isReversibleLocal, isSymmetric, isSwapped, issUncallLayout);
	IntermediateModel * im = new IntermediateModelIns(_insErrorCheck, id, next, previous);

	memory.saveObject(id->toString(), im);
}

} /* namespace dsl */
