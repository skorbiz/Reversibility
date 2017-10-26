/*
 * IoComparisonMonitored.cpp
 *
 *  Created on: May 23, 2015
 *      Author: josl
 */

#include "IoComparisonMonitored.hpp"

namespace dsl
{

IoComparisonMonitored::IoComparisonMonitored(const dsl::IOPorts & ioports, const dsl::Switch & expectedValues, std::shared_ptr<DigitalIOGeneralInterfaceProxy> DGIP) :
	_DGIP( DGIP )
{
	addArgument(ioports);
	addArgument(expectedValues);
}


IoComparisonMonitored::~IoComparisonMonitored()
{
}

// **********************************
// *** Build functions

void IoComparisonMonitored::addArgument(const dsl::IOPorts & arg)
{
	std::vector<int> ports = arg.get();
	_ioports = std::vector<int8_t>(ports.begin(), ports.end());
}

void IoComparisonMonitored::addArgument(const dsl::Switch & arg)
{
	int value = 0;
    switch ( arg.get() )
    {
       case dsl::Switch::on:		value = 1;		break;
       case dsl::Switch::off:		value = 0;		break;
       case dsl::Switch::toggle:	std::cerr << "Can't test if io was toggled" << std::endl; exit(-1);	break;
       default:						std::cerr << "Error in IoComparison" << std::endl; exit(-1);
    }

	for(unsigned int i = 0; i < _ioports.size(); i++)
		_expectedValues.push_back(value);
}

// **********************************
// *** Overwritten functions

bool IoComparisonMonitored::evaluate()
{
	std::vector<uint8_t> actualValues = _DGIP->getDigitalInput(_ioports);

	for(unsigned int i = 0; i < actualValues.size(); i++)
		if(_expectedValues[i] != actualValues[i])
			return false;

	return true;
}

} /* namespace dsl */
