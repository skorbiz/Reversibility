/*
 * Element.cpp
 *
 *  Created on: Nov 24, 2014
 *      Author: josl
 */

#include "Element.hpp"

namespace dsl
{

Element::Element() :
	_isReversible(true),
	_reverseCounterpart(nullptr)
{
}

Element::~Element()
{
}

void Element::setIsReversible(bool isReversible)
{
	_isReversible = isReversible;
}

void Element::setReverseCounterpart(std::shared_ptr<dsl::Element> reversecounterpart)
{
	_reverseCounterpart = reversecounterpart;
}

std::shared_ptr<dsl::Element> Element::getReverseCounterpart()
{
	return _reverseCounterpart;
}




// ******************************************************************
// **** VIRTUAL FUNCTIONS, MAY BE OVERWRITTEN ***********************
// ******************************************************************



} /* namespace dsl */
