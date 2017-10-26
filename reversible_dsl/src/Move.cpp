/*
 * Move.cpp
 *
 *  Created on: Feb 7, 2017
 *      Author: josl
 */

#include "Move.h"

#include <rw/kinematics/Kinematics.hpp>
#include "colors.hpp"
#include <assert.h>

Move::Move(std::shared_ptr<robot_movement_interface::RobworkInterface> iiwa,
		std::shared_ptr<Planner> planner,
		rw::models::WorkCell::Ptr wc,
		rw::models::Device::Ptr device,
		std::shared_ptr<rw::kinematics::Frame>toolFrame) :
			iiwa(iiwa),
			planner(planner),
			wc(wc),
			device(device),
			toolFrame(toolFrame.get())
{
}

Move::~Move()
{
}


bool Move::plan(rw::math::Transform3D<> baseTtool, rw::kinematics::State state)
{
	path = createPath(baseTtool, state);
	if(path.size() != 0)
		return true;
	return false;
}

/////////////////////////////////////////////////////////////////////////////////////////
// MOVE FUNCTIONS


void Move::move_forward()
{
//	ROS_INFO("Moving forward");
	iiwa->movePtp(path);
}

void Move::move_reverse()
{
//	ROS_INFO("Moving Backwards");
	std::vector<rw::math::Q > reverse_path = path;
	reverse_path.pop_back();
	std::reverse(reverse_path.begin(),reverse_path.end());
	iiwa->movePtp(reverse_path);
}

void Move::move_retract_from_fixture()
{
//	ROS_INFO_STREAM("Moving away from fixture");
	rw::math::Q q = iiwa->getQ();
	if(q[1] > 0)
		q[1] = q[1] - 10.0*(M_PI/180.0);
	else
		q[1] = q[1] + 10.0*(M_PI/180.0);
	iiwa->movePtp(q);
}


/////////////////////////////////////////////////////////////////////////////////////////
// GET & SET

std::vector<rw::math::Q> Move::get_path()
{
	return path;
}

rw::math::Q Move::get_endQ()
{
	assert(path.size() > 0);
	return path.back();
}

/////////////////////////////////////////////////////////////////////////////////////////
// PATH PLANNING

std::vector<rw::math::Q> Move::createPath(rw::math::Transform3D<> baseTtool, rw::kinematics::State state)
{
	std::vector<rw::math::Q> path;
	rw::math::Transform3D<> toolTend = rw::kinematics::Kinematics::frameTframe(toolFrame, device->getEnd(), state);
	for (std::size_t i = 0; i < 10; i++)
	{
		path = planner->plan(baseTtool*toolTend,state);
		if (path.size() != 0)
			return path;
//		ROS_WARN("No path found");
	}
	return path;
}
