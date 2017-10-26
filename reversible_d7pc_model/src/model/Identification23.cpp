/*
 * Identification23.cpp
 *
 *  Created on: Jun 15, 2017
 *      Author: josl
 */

#include <model/Identification23.h>

namespace model {

Identification23::Identification23(std::string sequence, int instance, int number, int depth ) :
		_origin(sequence),
		_instance(std::to_string(instance)),
		_number(std::to_string(number)),
		_depth(depth)
{
}

Identification23::Identification23(std::string sequence, int instance, std::string number, int depth ) :
	_origin(sequence),
	_instance(std::to_string(instance)),
	_number(number),
	_depth(depth)
{
}

Identification23::~Identification23()
{
}

std::string Identification23::getOrigin() const
{
	return _origin;
}

std::string Identification23::getInstance() const
{
	return _instance;
}

std::string Identification23::getGrouping() const
{
	return _origin + " [" + _instance + ".grouping]";
}

std::string Identification23::getNumber() const
{
	return _number;
}

int Identification23::getDepth() const
{
	return _depth;
}


std::string Identification23::toString() const
{
	return _origin + " [" + _instance + "." + _number + "]";
}

Identification23::operator std::string() const
{
	return toString();
};

std::ostream& operator<<(std::ostream& os, const Identification23& id)
{
	os << id.toString();
	return os;
}
} /* namespace model */
