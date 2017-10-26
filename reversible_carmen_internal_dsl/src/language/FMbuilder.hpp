/*
 * FMbuilder.hpp
 *
 *  Created on: Mar 13, 2014
 *      Author: josl
 */

#ifndef FMBUILDER_HPP_
#define FMBUILDER_HPP_


#include <iostream>
#include <../src/model/basemodel/argument/quantities/ForceModeArgument.hpp>

namespace dsl
{

class FMbuilder
{

private:
	class TaskFrameBuilder;
	class ComplianceBuilder;
	class NonComplianceBuiler;

	ForceModeArgument 	* _current;
	TaskFrameBuilder    * _taskFrameBuilderCurrent;
	ComplianceBuilder   * _complianceBuilderCurrent;
	NonComplianceBuiler * _nonComplianceBuilerCurrent;

public:
	FMbuilder();
	~FMbuilder();

	ForceModeArgument * createForceModeArgument();
	TaskFrameBuilder  & taskFrame();
	ComplianceBuilder & compliance();
};

// ******************************************************************
// **** INNER CLASS - TASKFRAMEBUIDER *******************************
// ******************************************************************

class FMbuilder::TaskFrameBuilder
{

public:
	TaskFrameBuilder(ForceModeArgument & forcemodeObj, ComplianceBuilder & complianceBuilder);
	~TaskFrameBuilder();
	TaskFrameBuilder & x(double m);
	TaskFrameBuilder & y(double m);
	TaskFrameBuilder & z(double m);
	TaskFrameBuilder & rx(double rad);
	TaskFrameBuilder & ry(double rad);
	TaskFrameBuilder & rz(double rad);
	ComplianceBuilder & compliance();

private:
	ForceModeArgument * forcemodeObj;
	ComplianceBuilder * complianceBuilder;
};





class FMbuilder::ComplianceBuilder{
public:
	ComplianceBuilder(ForceModeArgument & forcemodeObj, NonComplianceBuiler & nonComplianceBuilder);
	~ComplianceBuilder();

	ComplianceBuilder & x();
	ComplianceBuilder & y();
	ComplianceBuilder & z();
	ComplianceBuilder & rx();
	ComplianceBuilder & ry();
	ComplianceBuilder & rz();
	ComplianceBuilder & setForce(double newton_newtonmeter);
	ComplianceBuilder & maxSpeed(double mmPRSec_degPRsec);
	NonComplianceBuiler & nonCompliantOptions();

private:
	ForceModeArgument * forcemodeObj;
	NonComplianceBuiler * nonComplianceBuilder;
	ForceModeArgument::axis_t axis;
};





class FMbuilder::NonComplianceBuiler{
public:
	NonComplianceBuiler(ForceModeArgument & forcemodeObj);
	~NonComplianceBuiler();

	NonComplianceBuiler &  x();
	NonComplianceBuiler &  y();
	NonComplianceBuiler &  z();
	NonComplianceBuiler & rx();
	NonComplianceBuiler & ry();
	NonComplianceBuiler & rz();
	NonComplianceBuiler & compensateForForce(double newton_newtonmeter);
	NonComplianceBuiler & maxDiviation(double mm_deg);

private:
	ForceModeArgument * forcemodeObj;
	ForceModeArgument::axis_t axis;
};




} /* namespace dsl */
#endif /* FMBUILDER_HPP_ */
