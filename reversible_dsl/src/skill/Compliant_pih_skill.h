/*
 * compliant_pih_skill.h
 *
 *  Created on: Mar 20, 2017
 *      Author: jpb
 */

#ifndef D7PC_ROS_REVERSIBLE_DSL_SRC_SKILL_COMPLIANT_PIH_SKILL_H_
#define D7PC_ROS_REVERSIBLE_DSL_SRC_SKILL_COMPLIANT_PIH_SKILL_H_

#include <vector>

#include <rw/math.hpp>

#include <d7cp_proxys/robot_movement_interface/RobworkInterface.h>
#include "../D7cpWorkcell.h"

class Compliant_pih_skill {
public:
	Compliant_pih_skill(std::shared_ptr<robot_movement_interface::RobworkInterface> robot, std::shared_ptr<D7cpWorkcell> wc, rw::math::Transform3D<> toolTobject);

	struct parameters {
		double tilt_angle;
		double length_start_traj;
		double length_insertion_traj;
		double length_push;
		double length_top_rot;
		double length_rot_end;
	};

	static constexpr parameters d7pc_innerstructure = {
			.tilt_angle = 7 * rw::math::Pi/180,
			.length_start_traj = 0.03,
			.length_insertion_traj = 0.02,
			.length_push = 0.01,
			.length_top_rot = 0.01,
			.length_rot_end = 0.02
	};

	void run_skill(const parameters & param);

	std::vector<rw::math::Transform3D<> > generateTrajectory_baseTtool(const parameters & param);
	std::vector<rw::math::Transform3D<> > generateTrajectory(parameters param);

private:
	std::shared_ptr<robot_movement_interface::RobworkInterface> mRobot;
	std::shared_ptr<D7cpWorkcell> mWc;
	rw::math::Transform3D<> mmaleTtool;

	rw::math::Transform3D<> rotate_around_point(rw::math::Transform3D<> T, rw::math::Vector3D<> rot_point, rw::math::RPY<> angle);
};

#endif /* D7PC_ROS_REVERSIBLE_DSL_SRC_SKILL_COMPLIANT_PIH_SKILL_H_ */
