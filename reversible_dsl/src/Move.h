/*
 * Move.h
 *
 *  Created on: Feb 7, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DSL_SRC_MOVE_H_
#define D7PC_ROS_REVERSIBLE_DSL_SRC_MOVE_H_

#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>
#include <d7cp_proxys/robot_movement_interface/RobworkInterface.h>
#include "Planner.h"

#include <memory.h>

class Move
{

public:
	Move(std::shared_ptr<robot_movement_interface::RobworkInterface> iiwa,
			std::shared_ptr<Planner> planner,
			rw::models::WorkCell::Ptr wc,
			rw::models::Device::Ptr device,
			std::shared_ptr<rw::kinematics::Frame>toolFrame
			);
	virtual ~Move();

	bool plan(rw::math::Transform3D<> baseTtool, rw::kinematics::State state);

	void move_forward();
	void move_reverse();
	void move_retract_from_fixture();

	rw::math::Q get_endQ();
	std::vector<rw::math::Q> get_path();


private:
	std::vector<rw::math::Q> createPath(rw::math::Transform3D<> baseTtool, rw::kinematics::State state);

private:
	std::shared_ptr<robot_movement_interface::RobworkInterface> iiwa;
	std::shared_ptr<Planner> planner;

	const rw::models::WorkCell::Ptr wc;
	const rw::models::Device::Ptr device;
	const rw::kinematics::Frame* const toolFrame;

	std::vector<rw::math::Q> path;

};

#endif /* D7PC_ROS_REVERSIBLE_DSL_SRC_MOVE_H_ */
