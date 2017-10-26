/*
 * Move2Q.h
 *
 *  Created on: Jan 31, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DSL_SRC_MOVE2Q_H_
#define D7PC_ROS_REVERSIBLE_DSL_SRC_MOVE2Q_H_

#include <ros/ros.h>
#include <rw/math/Q.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>

//#include <d7cp_proxys/iiwa/RobworkInterface.h>
#include <d7cp_proxys/robot_movement_interface/RobworkInterface.h>

#include "Planner.h"

class Move2Q
{

public:
	Move2Q(std::shared_ptr<robot_movement_interface::RobworkInterface> iiwa,
			std::shared_ptr<Planner> planner);
	virtual ~Move2Q();

	bool move(rw::math::Q qTarget, rw::kinematics::State state) const;


private:
	std::shared_ptr<robot_movement_interface::RobworkInterface> iiwa;
	std::shared_ptr<Planner> planner;

};

#endif /* D7PC_ROS_REVERSIBLE_DSL_SRC_MOVE2Q_H_ */
