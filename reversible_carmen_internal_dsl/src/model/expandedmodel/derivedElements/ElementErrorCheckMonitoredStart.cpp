/*
 * ElementErrorCheckMonitoredStart.cpp
 *
 *  Created on: May 23, 2015
 *      Author: josl
 */

#include "ElementErrorCheckMonitoredStart.hpp"

namespace dsl {

ElementErrorCheckMonitoredStart::ElementErrorCheckMonitoredStart(dsl::ConditionMonitored * condition) :
				_insErrorCheckMS(nullptr),
				_condition(condition)
{
}

ElementErrorCheckMonitoredStart::~ElementErrorCheckMonitoredStart()
{
}

void ElementErrorCheckMonitoredStart::buildIntermediateModel(direction dir, bool isUncallLayout, bool isReversible, dsl::Identification::Ptr id, dsl::Identification::Ptr next, dsl::Identification::Ptr previous, MemoryModel<dsl::IntermediateModel*> & memory)
{
	bool switchedDirection = false;
	if(dir == direction::REVERSE)
		switchedDirection = true;

	bool isReversibleLocal = isReversible and _isReversible;
	bool isSymmetric = false;
	bool isSwapped = switchedDirection;
	bool issUncallLayout = isUncallLayout;

	_insErrorCheckMS = std::make_shared<dsl::ErrorCheckMonitoredStart>(*id, _condition, isReversibleLocal, isSymmetric, isSwapped, issUncallLayout);

	IntermediateModel * im = new IntermediateModelIns(_insErrorCheckMS, id, next, previous);
	memory.saveObject(id->toString(), im);
}


} /* namespace dsl */


