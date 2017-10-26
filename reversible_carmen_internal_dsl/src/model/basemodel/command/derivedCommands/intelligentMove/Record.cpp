/*
 * Record.cpp
 *
 *  Created on: Feb 29, 2016
 *      Author: josl
 */

#include "Record.hpp"

namespace dsl {
namespace intelligentmove {

Record::Record() :
		qStart(6,0,0,0,0,0,0),
		qEnd(6,0,0,0,0,0,0),
		speed(0),
		acceleration(0),
		type(MoveType::move),
		isActive(false),
		successfullMove(false),
		note(" ")
{
}

Record::~Record()
{
}


} /* namespace intelligentmove */
} /* namespace dsl */
