/*
 * MoveLin.cpp
 *
 *  Created on: Mar 10, 2017
 *      Author: josl
 */

#include "MoveLin.h"

MoveLin::MoveLin(std::shared_ptr<robot_movement_interface::RobworkInterface> iiwa,
		std::shared_ptr<Planner> planner,
		std::shared_ptr<rw::kinematics::Frame>toolFrame) :
		iiwa(iiwa),
		planner(planner),
		toolFrame(toolFrame),
		expected_reduncy(0)
{
}

MoveLin::~MoveLin()
{
}

bool MoveLin::is_in_reach(rw::math::Transform3D<> baseTtool, rw::kinematics::State stateFrom)
{
	std::vector<rw::math::Q> path = to_path(baseTtool, stateFrom);
	if (path.size() != 0)
		return true;
	return false;
}

std::vector<rw::math::Q> MoveLin::to_path(rw::math::Transform3D<> baseTtool, rw::kinematics::State stateFrom)
{
	std::vector<rw::math::Q> path;
	for (std::size_t i = 0; i < 10 && path.size() == 0; i++)
	{
		path = planner->solve(baseTtool,toolFrame.get(),stateFrom);
		if (path.size() != 0)
			break;
	}
	return path;
}

void MoveLin::set_expected_redundency(double r)
{
	expected_reduncy = r;
}


void MoveLin::move_forward(rw::math::Transform3D<> baseTtool)
{
//	ROS_INFO("Moving forward");
	double redundency = iiwa->getQ()[2];

//	std::cout << "EXPECTED REDUNDENCY: " << expected_reduncy << std::endl;
//	std::cout << "ACTUALE REDUNDENCY : " << redundency << std::endl;

	iiwa->moveLinRedundency(baseTtool,redundency);
}
