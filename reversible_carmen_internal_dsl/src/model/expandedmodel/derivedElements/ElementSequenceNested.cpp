/*
 * ElementSequenceNested.cpp
 *
 *  Created on: Dec 24, 2014
 *      Author: josl
 */

#include "ElementSequenceNested.hpp"

namespace dsl
{

ElementSequenceNested::ElementSequenceNested(std::shared_ptr<dsl::Sequence> nestedSequence,  bool reverseLayout, bool uncallLayout) :
	_nestedSequence(nestedSequence),
	_reverseLayout(reverseLayout),
	_uncallLayout(uncallLayout)
{
//	gen::dExtendConstruction << this << " :: ElementSequenceNested :: Constructor of EllementSequenceNested" << std::endl;
}

ElementSequenceNested::~ElementSequenceNested()
{
}

void ElementSequenceNested::buildIntermediateModel(direction dir, bool uncallLayout, bool isReversible, dsl::Identification::Ptr id, dsl::Identification::Ptr next, dsl::Identification::Ptr previous, MemoryModel<dsl::IntermediateModel*> & memory)
{
	direction aDir = dir;
	if(_reverseLayout)
		aDir = switchDirection(dir);

	bool uncall = _uncallLayout;

	if(uncallLayout)
		uncall = !_uncallLayout;

	if(getReverseCounterpart() != nullptr)
		uncall = false;

	bool isReversibleLocal = isReversible and _isReversible;

	_nestedSequence->buildIntermediateModel(aDir, uncall, isReversibleLocal, id, next, previous, memory);
}

Element::direction ElementSequenceNested::switchDirection(direction dir)
{
	if(dir == direction::FORWARD)
		return direction::REVERSE;
	else
		return direction::FORWARD;
}



} /* namespace dsl */
