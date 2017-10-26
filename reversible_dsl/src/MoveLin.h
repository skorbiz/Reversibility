/*
 * MoveLin.h
 *
 *  Created on: Mar 10, 2017
 *      Author: josl
 */

#ifndef REVERSIBLE_DSL_SRC_PREPLANNER_MOVELIN_H_
#define REVERSIBLE_DSL_SRC_PREPLANNER_MOVELIN_H_

#include <memory.h>
#include <d7cp_proxys/robot_movement_interface/RobworkInterface.h>
#include "Planner.h"

class MoveLin
{

public:
	MoveLin(std::shared_ptr<robot_movement_interface::RobworkInterface> iiwa,
			std::shared_ptr<Planner> planner,
			std::shared_ptr<rw::kinematics::Frame>toolFrame);
	virtual ~MoveLin();

	bool is_in_reach(rw::math::Transform3D<> baseTtool, rw::kinematics::State state);
	std::vector<rw::math::Q> to_path(rw::math::Transform3D<> baseTtool, rw::kinematics::State state);
	void move_forward(rw::math::Transform3D<> baseTtool);

	void set_expected_redundency(double r);


private:
	std::shared_ptr<robot_movement_interface::RobworkInterface> iiwa;
	std::shared_ptr<Planner> planner;
	std::shared_ptr<rw::kinematics::Frame> toolFrame;

	double expected_reduncy;

};

#endif /* REVERSIBLE_DSL_SRC_PREPLANNER_MOVELIN_H_ */
