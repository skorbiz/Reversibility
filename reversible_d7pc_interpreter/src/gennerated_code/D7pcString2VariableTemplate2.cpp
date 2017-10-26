/*
 * D7pcString2Variable.cpp
 *
 *  Created on: Jun 9, 2017
 *      Author: josl
 */

#include <gennerated_code/D7pcString2Variable.h>

namespace edsl {

D7pcString2Variable::D7pcString2Variable() :
	wc(std::make_shared<D7cpWorkcell>())
{
}

D7pcString2Variable::~D7pcString2Variable()
{
}

std::shared_ptr<rw::kinematics::Frame> D7pcString2Variable::toFrame(std::string name)
{
	 $$var1
	 $$var2

	if(name == "wc.assemblyFrame")
		 return wc->assemblyFrame;
	if(name == "wc.toolFrame")
		 return wc->toolFrame;
	if(name == "wc.baseFrame")
		 return wc->baseFrame;

	std::cerr << __PRETTY_FUNCTION__ << std::endl;
	std::cerr << "No frame with name: " << name << std::endl;
	exit(-1);
}

rw::math::Q D7pcString2Variable::toQ(std::string name)
{
	 $$var3
	 $$var4

	if(name == "wc.qInspectOuterShell")
		 return wc->qInspectOuterShell;
	if(name == "wc.qInspectInnerStructure")
		 return wc->qInspectInnerStructure;
	if(name == "wc.qInspectScrewPart")
		 return wc->qInspectScrewPart;

	std::cerr << __PRETTY_FUNCTION__ << std::endl;
	std::cerr << "No Q with name: " << name << std::endl;
	exit(-1);
}


} /* namespace edsl */

