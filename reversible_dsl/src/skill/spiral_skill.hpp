#ifndef SPIRAL_SKILL_SPIRAL_SKILL_HPP_
#define SPIRAL_SKILL_SPIRAL_SKILL_HPP_

#include <memory>

#include <vector>
#include <rw/math/Transform3D.hpp>

#include <d7cp_proxys/robot_movement_interface/RobworkInterface.h>
#include "../D7cpWorkcell.h"

class SpiralSkill {
public:
	SpiralSkill(std::shared_ptr<robot_movement_interface::RobworkInterface> robot, std::shared_ptr<D7cpWorkcell> wc, rw::math::Transform3D<> toolTobject);

	struct parameters {
		double length_start_traj;
		double d_path;
		double r;
		double n;
		double sx;
		double sy;
		double length_push;
	};

	bool run_spiral_skill(parameters param);

	static constexpr parameters d7pc_pin_parameters = {
			.length_start_traj = 0.04,
			.d_path = 1.0,
			.r = 1/3.0,
			.n = 3,
			.sx = 0.005,
			.sy = 0.005,
			.length_push = -0.003
	};

	static constexpr parameters sim_optimized_d7pc_parameters = {
			.length_start_traj = 0.04,
			.d_path = 120* rw::math::Pi /180.,
			.r = 0.0001,
			.n = (120* rw::math::Pi /180.0) * (20/(2*rw::math::Pi)),
			.sx = 1,
			.sy = 1,
			.length_push = -0.005 //-0.003
	};


private:
	std::vector<rw::math::Transform3D<> > generateTrajectory_baseTtool(parameters param) const;
	std::vector<rw::math::Transform3D<> > generateTrajectory(parameters param) const;
	bool is_pin_in_hole(rw::math::Transform3D<> baseTtool) const;

private:
	std::shared_ptr<robot_movement_interface::RobworkInterface> mController;
	std::shared_ptr<D7cpWorkcell> mWc;
	rw::math::Transform3D<> mmaleTtool;
};

#endif
