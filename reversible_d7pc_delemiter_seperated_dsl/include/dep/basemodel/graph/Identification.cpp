/*
 * Identification.cpp
 *
 *  Created on: Mar 22, 2017
 *      Author: josl
 */

#include "Identification.h"

namespace dsl {
namespace basemodel {
namespace graph {

Identification::Identification(std::string sequence, int instance, int number, int depth ) :
		_origin(sequence),
		_instance(std::to_string(instance)),
		_number(std::to_string(number)),
		_depth(depth)
{
}

Identification::Identification(std::string sequence, int instance, std::string number, int depth ) :
	_origin(sequence),
	_instance(std::to_string(instance)),
	_number(number),
	_depth(depth)
{
}

Identification::~Identification()
{
}

std::string Identification::getOrigin() const
{
	return _origin;
}

std::string Identification::getInstance() const
{
	return _instance;
}

std::string Identification::getGrouping() const
{
	return _origin + " [" + _instance + ".grouping]";
}

std::string Identification::getNumber() const
{
	return _number;
}

int Identification::getDepth() const
{
	return _depth;
}


std::string Identification::toString() const
{
	return _origin + " [" + _instance + "." + _number + "]";
}

Identification::operator std::string() const
{
	return toString();
};

std::ostream& operator<<(std::ostream& os, const Identification& id)
{
	os << id.toString();
	return os;
}

} /* namespace graph */
} /* namespace basemodel */
} /* namespace dsl */
