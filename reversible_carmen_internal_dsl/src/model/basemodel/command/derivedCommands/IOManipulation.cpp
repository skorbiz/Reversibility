/*
 * IOManipulation.cpp
 *
 *  Created on: Nov 6, 2014
 *      Author: josl
 */

#include "IOManipulation.hpp"

namespace dsl {

IOManipulation::IOManipulation(std::shared_ptr<DigitalIOGeneralInterfaceProxy> DGIP, std::vector<int> ioports, dsl::switch_t::switch_t type) :
	_DGIP(DGIP),
	_type(type),
	_ioports(ioports.begin(), ioports.end()),
	_hadRedundantIOCommands(false),
	_hadRedundantIOCommandsSwappedTemp(false)
{
	int valueForward = -1;
	int valueBackward = -1;

	if(_type == dsl::switch_t::switch_t::ON){
		valueForward = 1;
		valueBackward = 0;
	}
	else if(_type == dsl::switch_t::switch_t::OFF){
		valueForward = 0;
		valueBackward = 1;
	}

	for(unsigned int i = 0; i < _ioports.size(); i++)
	{
		_valuesForward.push_back(valueForward);
		_valuesBackwards.push_back(valueBackward);
	}

}

IOManipulation::~IOManipulation()
{
}

void IOManipulation::execute()
{
	execute(_valuesForward);
}


void IOManipulation::executeBackwards()
{
	execute(_valuesBackwards);
}


void IOManipulation::execute(std::vector<uint8_t>& vec)
{
	if(_type == switch_t::switch_t::UNSPECIFIED)
	{
		std::cerr << "Can't execute IOManipulation with unspecified operations" << std::endl;
		exit(-1);
	}

	if(_type == switch_t::switch_t::TOGGLE)
	{
		vec = _DGIP->getDigitalOutput(_ioports);
		for(unsigned int i = 0; i < vec.size(); i++)
			vec[i] = std::abs(vec[i]-1); //One-liner for inverting bits
	}
	_DGIP->setDigitalOutput(_ioports,vec);
}

void IOManipulation::stateUpdate(dsl::State & state)
{
	for(unsigned int i = 0; i < _ioports.size(); i++)
		stateUpdate(i, state);
}

void IOManipulation::stateUpdateSwapped(dsl::State & state)
{
	for(unsigned int i = 0; i < _ioports.size(); i++)
		stateUpdateSwapped(i, state);
}

void IOManipulation::stateUpdate(int index, dsl::State & state)
{
	bool isRedundant = false;

	if( state.getIO( _ioports[index]) == _valuesForward[index] )
		isRedundant = true;

	if( isRedundant)
	{
		_hadRedundantIOCommands = true;
		_valuesBackwards[index] = std::abs(_valuesBackwards[index]-1); //One-liner for inverting bits
	}

	state.update(_ioports[index], _valuesForward[index]);

	deb << "IOMaipulation state updated :: ";
	deb << "IOport: " << (int) _ioports[index] << " :: ";
	if(isRedundant)
		deb << "IO detected as redundant";
	deb << std::endl;
}


void IOManipulation::stateUpdateSwapped(int index, dsl::State & state)
{
	if( state.getIO( _ioports[index]) == _valuesBackwards[index] )
		_hadRedundantIOCommandsSwappedTemp = true;
	state.update(_ioports[index], _valuesBackwards[index]);
}


std::string IOManipulation::getType()
{
	return "io";
}

std::string IOManipulation::getArgumentForward() const
{
	std::stringstream res;
	res << "iop: " << (int) _ioports[0];
	res <<" val: " << (int) _valuesForward[0];
	res << (_hadRedundantIOCommandsSwappedTemp ? " - rSwapped" : "");
	return res.str();
}

std::string IOManipulation::getArgumentBackward() const
{
	std::stringstream res;
	res << "iop: " << (int) _ioports[0];
	res <<" val: " << (int) _valuesBackwards[0];
	res << (_hadRedundantIOCommands ? " - redundant" : "");
	return res.str();
}



} /* namespace dsl */
