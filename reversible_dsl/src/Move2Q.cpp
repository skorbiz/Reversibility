/*
 * Move2Q.cpp
 *
 *  Created on: Jan 31, 2017
 *      Author: josl
 */

#include "Move2Q.h"
#include <rw/kinematics/State.hpp>


Move2Q::Move2Q(std::shared_ptr<robot_movement_interface::RobworkInterface> iiwa, std::shared_ptr<Planner> planner) :
	iiwa(iiwa),
	planner(planner)
{
}

Move2Q::~Move2Q()
{
}

bool Move2Q::move(rw::math::Q q, rw::kinematics::State state) const
{
	std::vector<rw::math::Q> path;
	path = planner->plan(q,state);

	if (path.size() == 0)
		return false;

	iiwa->movePtp(path);

//	for (std::size_t i = 0; i < path.size(); i++)
//	{
//		iiwa->movePtp(path[i]);
//	}
	return true;
}





