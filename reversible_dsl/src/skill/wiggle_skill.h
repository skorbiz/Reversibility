/*
 * wiggleskill.h
 *
 *  Created on: Mar 20, 2017
 *      Author: jpb
 */

#ifndef D7PC_ROS_REVERSIBLE_DSL_SRC_SKILL_WIGGLE_SKILL_H_
#define D7PC_ROS_REVERSIBLE_DSL_SRC_SKILL_WIGGLE_SKILL_H_

#include <vector>

#include <rw/math/Transform3D.hpp>

#include <d7cp_proxys/robot_movement_interface/RobworkInterface.h>
#include "../D7cpWorkcell.h"

class WiggleSkill {
public:
	WiggleSkill(std::shared_ptr<robot_movement_interface::RobworkInterface> controller, std::shared_ptr<D7cpWorkcell> wc, rw::math::Transform3D<> toolTobject);

	struct parameters {
		double outer_angle; 		// given in radians.
		double push_depth; 			// in m
		int    wiggles_count;
		double depth_for_success;
	};

	static parameters defaultParam();

	bool run_skill(const parameters & param);

	std::vector<rw::math::Transform3D<> > generateTrajectory_baseTtool(const parameters & param);

	std::vector<rw::math::Transform3D<> > generateTrajectory(const parameters & param);

	bool successCriteria(const parameters & param, rw::math::Transform3D<> baseTtool);

private:
	std::shared_ptr<robot_movement_interface::RobworkInterface> mController;
	std::shared_ptr<D7cpWorkcell> mWc;
	rw::math::Transform3D<> mmaleTtool;
};

#endif /* D7PC_ROS_REVERSIBLE_DSL_SRC_SKILL_WIGGLE_SKILL_H_ */
