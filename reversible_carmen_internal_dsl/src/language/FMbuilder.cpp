/*
 * FMbuilder.cpp
 *
 *  Created on: Mar 13, 2014
 *      Author: josl
 */

#include "FMbuilder.hpp"


namespace dsl
{

FMbuilder::FMbuilder()
{
	_current = NULL;
	_taskFrameBuilderCurrent = NULL;
	_complianceBuilderCurrent = NULL;
	_nonComplianceBuilerCurrent = NULL;
}


FMbuilder::~FMbuilder()
{
}


ForceModeArgument * FMbuilder::createForceModeArgument()
{
	_current = new ForceModeArgument();

	_nonComplianceBuilerCurrent = new NonComplianceBuiler(*_current);
	_complianceBuilderCurrent = new ComplianceBuilder( *_current, *_nonComplianceBuilerCurrent);
	_taskFrameBuilderCurrent = new TaskFrameBuilder( *_current, *_complianceBuilderCurrent);

	_taskFrameBuilderCurrent->~TaskFrameBuilder();
	_complianceBuilderCurrent->~ComplianceBuilder();
	_nonComplianceBuilerCurrent->~NonComplianceBuiler();


	return _current;

}

FMbuilder::TaskFrameBuilder & FMbuilder::taskFrame()
{
	return * _taskFrameBuilderCurrent;
}

FMbuilder::ComplianceBuilder & FMbuilder::compliance()
{
	return * _complianceBuilderCurrent;
}

/*
 * *********************************************************************************************************************************************
 * *********************************************************************************************************************************************
 */

FMbuilder::TaskFrameBuilder::TaskFrameBuilder(ForceModeArgument & forcemodeObj, ComplianceBuilder & complianceBuilder)
{
	this->forcemodeObj = &forcemodeObj;
	this->complianceBuilder = &complianceBuilder;
}


FMbuilder::TaskFrameBuilder::~TaskFrameBuilder()
{
}


FMbuilder::TaskFrameBuilder & FMbuilder::TaskFrameBuilder::x(double m)
{
	forcemodeObj->setTaskFrame(ForceModeArgument::X,m);
	return * this;
}


FMbuilder::TaskFrameBuilder & FMbuilder::TaskFrameBuilder::y(double m)
{
	forcemodeObj->setTaskFrame(ForceModeArgument::Y,m);
	return * this;
}


FMbuilder::TaskFrameBuilder & FMbuilder::TaskFrameBuilder::z(double m)
{
	forcemodeObj->setTaskFrame(ForceModeArgument::Z,m);
	return * this;
}


FMbuilder::TaskFrameBuilder & FMbuilder::TaskFrameBuilder::rx(double rad)
{
	forcemodeObj->setTaskFrame(ForceModeArgument::RX,rad);
	return * this;
}


FMbuilder::TaskFrameBuilder & FMbuilder::TaskFrameBuilder::ry(double rad)
{
	forcemodeObj->setTaskFrame(ForceModeArgument::RY,rad);
	return * this;
}


FMbuilder::TaskFrameBuilder & FMbuilder::TaskFrameBuilder::rz(double rad)
{
	forcemodeObj->setTaskFrame(ForceModeArgument::RZ,rad);
	return * this;
}


FMbuilder::ComplianceBuilder & FMbuilder::TaskFrameBuilder::compliance()
{
	return * complianceBuilder;
}

/*
 * *********************************************************************************************************************************************
 * *********************************************************************************************************************************************
 */

FMbuilder::ComplianceBuilder::ComplianceBuilder(ForceModeArgument & forcemodeObj, NonComplianceBuiler & nonComplianceBuilder)
{
	this->forcemodeObj = &forcemodeObj;
	this->nonComplianceBuilder = &nonComplianceBuilder;
	this->axis = ForceModeArgument::axis_t::X;

	this->forcemodeObj->setType(ForceModeArgument::forcemodeType_t::POINT);
}


FMbuilder::ComplianceBuilder::~ComplianceBuilder()
{

}


FMbuilder::ComplianceBuilder & FMbuilder::ComplianceBuilder::x()
{
	axis = ForceModeArgument::axis_t::X;
	forcemodeObj->setSelectionVector(axis);
	return * this;
}


FMbuilder::ComplianceBuilder & FMbuilder::ComplianceBuilder::y()
{
	axis = ForceModeArgument::axis_t::Y;
	forcemodeObj->setSelectionVector(axis);
	return * this;
}


FMbuilder::ComplianceBuilder & FMbuilder::ComplianceBuilder::z()
{
	axis = ForceModeArgument::axis_t::Z;
	forcemodeObj->setSelectionVector(axis);
	return * this;
}


FMbuilder::ComplianceBuilder & FMbuilder::ComplianceBuilder::rx()
{
	axis = ForceModeArgument::axis_t::RX;
	forcemodeObj->setSelectionVector(axis);
	return * this;
}


FMbuilder::ComplianceBuilder & FMbuilder::ComplianceBuilder::ry()
{
	axis = ForceModeArgument::axis_t::RY;
	forcemodeObj->setSelectionVector(axis);
	return * this;
}


FMbuilder::ComplianceBuilder & FMbuilder::ComplianceBuilder::rz()
{
	axis = ForceModeArgument::axis_t::RZ;
	forcemodeObj->setSelectionVector(axis);
	return * this;
}


FMbuilder::ComplianceBuilder & FMbuilder::ComplianceBuilder::setForce(double newton_newtonmeter)
{
	forcemodeObj->setWrench(axis, newton_newtonmeter);
	return * this;
}


FMbuilder::ComplianceBuilder & FMbuilder::ComplianceBuilder::maxSpeed(double mmPRSec_degPRsec)
{
	forcemodeObj->setLimit(axis, mmPRSec_degPRsec);
	return * this;
}


FMbuilder::NonComplianceBuiler & FMbuilder::ComplianceBuilder::nonCompliantOptions()
{
	return * nonComplianceBuilder;
}




/*
 * *********************************************************************************************************************************************
 * *********************************************************************************************************************************************
 */

FMbuilder::NonComplianceBuiler::NonComplianceBuiler(ForceModeArgument & forcemodeObj)
{
	this->forcemodeObj = &forcemodeObj;
	this->axis = ForceModeArgument::axis_t::X;
}


FMbuilder::NonComplianceBuiler::~NonComplianceBuiler()
{

}


FMbuilder::NonComplianceBuiler & FMbuilder::NonComplianceBuiler::x()
{
	axis = ForceModeArgument::axis_t::X;
	return * this;
}


FMbuilder::NonComplianceBuiler & FMbuilder::NonComplianceBuiler::y()
{
	axis = ForceModeArgument::axis_t::Y;
	return * this;
}


FMbuilder::NonComplianceBuiler & FMbuilder::NonComplianceBuiler::z()
{
	axis = ForceModeArgument::axis_t::Z;
	return * this;
}


FMbuilder::NonComplianceBuiler & FMbuilder::NonComplianceBuiler::rx()
{
	axis = ForceModeArgument::axis_t::RX;
	return * this;
}


FMbuilder::NonComplianceBuiler & FMbuilder::NonComplianceBuiler::ry()
{
	axis = ForceModeArgument::axis_t::RY;
	return * this;
}


FMbuilder::NonComplianceBuiler & FMbuilder::NonComplianceBuiler::rz()
{
	axis = ForceModeArgument::axis_t::RZ;
	return * this;
}


FMbuilder::NonComplianceBuiler & FMbuilder::NonComplianceBuiler::compensateForForce(double newton_newtonmeter)
{
	forcemodeObj->setWrench(axis, newton_newtonmeter);
	return * this;
}


FMbuilder::NonComplianceBuiler & FMbuilder::NonComplianceBuiler::maxDiviation(double mm_deg)
{
	forcemodeObj->setLimit(axis,mm_deg);
	return * this;
}


} /* namespace dsl */
