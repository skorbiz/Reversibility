/*
 * Update.cpp
 *
 *  Created on: Jun 22, 2017
 *      Author: josl
 */

#include <model/derivedInstructions/Update.h>
#include <iostream>

namespace model {

Update::Update(std::string variable) :
		var(variable)
{
}

Update::~Update()
{
}

ExecutionResult23 Update::execute()
{

//	if(args.size() == 1)
//	{
//		rw::kinematics::State& state = var.getRef();
//		rw::math::Q qTo = variables->q(args[0]);
//		wc->setQ(qTo, state);
//	}
//
//	else if(args.size() == 2)
//	{
//		rw::kinematics::State& state = var.getRef();
//		auto frame_move = variables->frames.getRef(args[0]);
//		auto frame_to = variables->frames.getRef(args[1]);
//		wc->attachTo(frame_move, frame_to, state);
//	}
//	else
//		std::cout << "ERROR IN " << __PRETTY_FUNCTION__ << std::endl;


	std::cout << __PRETTY_FUNCTION__ << std::endl;
	ExecutionResult23 r;
	return r;
}

ExecutionResult23 Update::executeBackwards()
{
	std::cout << __PRETTY_FUNCTION__ << std::endl;
	ExecutionResult23 r;
	return r;
}


} /* namespace model */
