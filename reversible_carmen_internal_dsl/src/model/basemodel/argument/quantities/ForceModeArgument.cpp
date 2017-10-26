/*
 * ForceModeArgument.cpp
 *
 *  Created on: Jan 20, 2015
 *      Author: josl
 */

#include "ForceModeArgument.hpp"

namespace dsl {

ForceModeArgument::ForceModeArgument()
{
	_type = forcemodeType_t::FRAME;
	_taskframe = {{0,0,0,0,0,0}};
	_wrench = {{0,0,0,0,0,0}};
	_limit = {{0,0,0,0,0,0}};
	_selectionvector = {{false,false,false,false,false,false}};
	_activation = false;
}

ForceModeArgument::ForceModeArgument(forcemodeType_t type, boost::array<double,6> taskframe, boost::array<double,6> wrench, boost::array<double,6> limit, boost::array<bool,6> selectionvector) :
		_type(type),
		_taskframe(taskframe),
		_wrench(wrench),
		_limit(limit),
		_selectionvector(selectionvector),
		_activation(true)
{
}

ForceModeArgument::~ForceModeArgument()
{
}

void ForceModeArgument::setType(forcemodeType_t type)
{
	this->_type = type;
}

void ForceModeArgument::setSelectionVector(axis_t axis)
{
	_selectionvector[axis] = true;
}

void ForceModeArgument::setTaskFrame(axis_t axis, double value)
{
	_taskframe[axis] = value;
}

void ForceModeArgument::setWrench(axis_t axis, double value)
{
	_wrench[axis] = value;
}

void ForceModeArgument::setLimit(axis_t axis, double value)
{
	_limit[axis] = value;
}

int ForceModeArgument::getType()
{

    switch ( _type )
    {
       case forcemodeType_t::POINT:	 	return 1; 		break;
       case forcemodeType_t::FRAME: 	return 2; 		break;
       case forcemodeType_t::MOTION: 	return 3; 		break;
       default:std::cerr << "Error in ForceModeArgument" << std::endl; exit(-1);
    }

    return 0;
}


boost::array<double,6> ForceModeArgument::getTaskFrame()
{
	return _taskframe;
}


boost::array<bool,6> ForceModeArgument::getSelectionVector()
{
	return _selectionvector;
}


boost::array<double,6> ForceModeArgument::getWrench()
{
	return _wrench;
}


boost::array<double,6> ForceModeArgument::getLimits()
{
	return _limit;
}

bool ForceModeArgument::getActivationSignal()
{
	return _activation;
}


void ForceModeArgument::print(std::ostream& os) const
{
    os << "forceModeArgument( ... )";
}


std::ostream& operator<<(std::ostream& os, const ForceModeArgument& fma)
{
    os << "forceModeArgument( ... )";
    return os;
}



// ******************************
// FINISHED BUILDS

ForceModeArgument ForceModeArgument::getDeativationArgument()
{
	return ForceModeArgument();
}

ForceModeArgument ForceModeArgument::getPushForward(double N)
{
	double offset = 0;
	ForceModeArgument::forcemodeType_t type 	= 		ForceModeArgument::forcemodeType_t::FRAME;
	boost::array<bool,6> selectionvector_ud 	= 		{{ true, false, false, false, false, false}};
	boost::array<double,6> limit_ud 			= 		{{ 0.02,   100,   100,    10,    10,   10 }};
	boost::array<double,6> wrench 				= 		{{ N+offset, 0,    -5,     0,     0,    0 }};
	boost::array<double,6> taskframe 			= 		{{    0,     0,     0,     0,     0,    0 }};
	return ForceModeArgument(type, taskframe, wrench, limit_ud, selectionvector_ud);
}

ForceModeArgument ForceModeArgument::getPushRight(double N)
{
	double offset = 20;
	ForceModeArgument::forcemodeType_t type 	= 		ForceModeArgument::forcemodeType_t::FRAME;
	boost::array<bool,6> selectionvector_lr 	= 		{{false, true, false, false, false, false}};
	boost::array<double,6> limit_lr 			= 		{{  100,   0.02,   100,    10,    10,   10 }};
	boost::array<double,6> wrench 				= 		{{    0, N+offset, 0,     0,     0,    0 }};
	boost::array<double,6> taskframe 			= 		{{    0,     0,    0,     0,     0,    0 }};
	return ForceModeArgument(type, taskframe, wrench, limit_lr, selectionvector_lr);
}

ForceModeArgument ForceModeArgument::getPushUp(double N)
{
	double offset = -20;
	ForceModeArgument::forcemodeType_t type 	= 		ForceModeArgument::forcemodeType_t::FRAME;
	boost::array<bool,6> selectionvector_ud 	= 		{{false, false, true, false, false, false}};
	boost::array<double,6> limit_ud				= 		{{  100,   100, 0.02,    10,    10,   10 }};
	boost::array<double,6> wrench 				= 		{{ 	  0,     0, N+offset, 0,     0,    0 }};
	boost::array<double,6> taskframe 			= 		{{    0,     0,    0,     0,     0,    0 }};
	return ForceModeArgument(type, taskframe, wrench, limit_ud, selectionvector_ud);
}

ForceModeArgument ForceModeArgument::getArgument(std::string id)
{
	std::cerr << "ForceModeArgument::getArgument(...) is not yet implemented" << std::endl;
	return getDeativationArgument();
}















} /* namespace dsl */
