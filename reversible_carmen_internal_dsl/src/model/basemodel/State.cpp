/*
 * State.cpp
 *
 *  Created on: Mar 30, 2015
 *      Author: josl
 */

#include "State.hpp"

namespace dsl
{

State::State() :
	_qGripper(-1)
{
	coutS.setPrefix("[state] ");
	coutS.setColorPrefix(dsl::color::CYAN);
}

//State::State(const State& s) :
//	_q(s._q),
//	_str(s._str),
//	_ioports(s._ioports.begin(),s._ioports.end()),
//	_iovalues(s._iovalues.begin(),s._iovalues.end())
//{
//}

State::~State()
{
}

rw::math::Q State::getQ() const
{
	return _q;
}

void State::update(rw::math::Q q)
{
//	coutS << "was: " << _q << std::endl;
//	coutS << "bec: " << q << std::endl;
	_q = q;
}

int State::getGripper() const
{
	return _qGripper;
}

void State::updateGripper(int q)
{
	std::string status;
	if(_qGripper == 0)
		status = "open ";
	if(_qGripper == 255)
		status = "closed ";

//	coutS << "was: " << status << _qGripper << std::endl;
//	coutS << "bec: " << status <<_qGripper << std::endl;
	_qGripper = q;
}

std::string State::getText() const
{
	return _str;
}

void State::update(std::string text)
{
	_str = text;
}

int State::getIO(int pin) const
{
	for(unsigned int i = 0; i<_ioports.size(); i++)
		if(_ioports[i] == pin)
			return _iovalues[i];
	return -1;
}

void State::update(int pin, int value)
{
//	coutS << "IO state update values: ";
//	coutS << "size: " << _ioports.size() << std::endl;
	for(unsigned int i = 0; i<_ioports.size(); i++)
		coutS << "p:" << (int) _ioports[i] << " v: " << (int) _iovalues[i] << "  ";
	coutS << std::endl;

	unsigned int i;
	for(i = 0; i<_ioports.size(); i++)
		if(_ioports[i] == pin)
			break;

	if(i < _ioports.size())
		_iovalues[i] = value;
	else
	{
		_ioports.push_back(pin);
		_iovalues.push_back(value);
	}


	for(unsigned int i = 0; i<_ioports.size(); i++)
		coutS << "p:" << (int) _ioports[i] << " v: " << (int) _iovalues[i] << "  ";
	coutS << std::endl;
}


} /* namespace dsl */
