/*
 * Switch.cpp
 *
 *  Created on: Nov 12, 2014
 *      Author: josl
 */

#include "Switch.hpp"

namespace dsl
{

Switch::Switch()
{
}

Switch::Switch(switch_t::switch_t value) : Datatype(value)
{
}

Switch::~Switch()
{
}


std::ostream& operator<<(std::ostream& os, const Switch& s)
{
    os << "Switch(";
    switch ( s.get() )
    {
       case switch_t::switch_t::OFF:		os << "OFF";			break;
       case switch_t::switch_t::ON:			os << "ON";				break;
       case switch_t::switch_t::TOGGLE:		os << "TOGGLE";			break;
       case switch_t::switch_t::UNSPECIFIED:os << "UNSPECIFIED";	break;
       default: os << "No value was specified";
    }
    os << ")";
    return os;
}




} /* namespace dsl */
