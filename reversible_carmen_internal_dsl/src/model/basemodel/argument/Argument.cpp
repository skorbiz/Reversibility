/*
 * Argument.cpp
 *
 *  Created on: Nov 1, 2014
 *      Author: josl
 */

#include "Argument.hpp"

namespace dsl
{

Argument::Argument()
{

}

Argument::~Argument()
{

}

void Argument::print(std::ostream& os) const
{
	os << "Argument with no overload";
}

//std::ostream& operator<<(std::ostream& os, Argument const& data)
//{
//	data.print(os);
//    return os;
//}


} /* namespace dsl */
