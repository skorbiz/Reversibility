/*
 * IOPorts.cpp
 *
 *  Created on: Nov 12, 2014
 *      Author: josl
 */

#include "IOPorts.hpp"

namespace dsl {

IOPorts::IOPorts()
{
}

IOPorts::IOPorts(int ioport)
{
	get().push_back(ioport);
}

IOPorts::IOPorts(std::initializer_list<int> ioports)
{
	for( auto elem : ioports )
	{
		get().push_back(elem);
	}

}

IOPorts::~IOPorts()
{
}

void IOPorts::append(int t)
{
	get().push_back(t);
}


std::ostream& operator<<(std::ostream& os, const IOPorts& i)
{
    os << "IOPorts( ";
    for(unsigned int j = 0; j < i.get().size(); j++ )
    	os << i.get()[j] << " ";
    os << ")";
    return os;
}


} /* namespace dsl */
