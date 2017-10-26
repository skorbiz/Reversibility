/*
 * WSG25Gripper.h
 *
 *  Created on: Feb 7, 2017
 *      Author: jpb
 */

#ifndef D7PC_ROS_CARMEN_IIWA_INCLUDE_CARMEN_IIWA_GRIPPER_WSG25GRIPPER_H_
#define D7PC_ROS_CARMEN_IIWA_INCLUDE_CARMEN_IIWA_GRIPPER_WSG25GRIPPER_H_

#include <memory>

#include "WSG25Proxy.h"

namespace gripper {

class WSG25Gripper {
public:
	typedef std::shared_ptr<WSG25Gripper> Ptr;

	WSG25Gripper(WSG25Proxy::Ptr wsg25proxy);
	virtual ~WSG25Gripper() {}

	void homing() { wsg25_->homing(); }
	bool grasp(double width, double speed);
	bool release(double width, double speed);

	bool move(double width, double speed);
	bool open(double speed);
	bool close(double speed);

	bool block_while_moving();
	bool atRest();
	bool set_force(double force);

	double getWidth();
	double getSpeed();
	double getForce();

	static constexpr double position_min =  0.0;	//mm
	static constexpr double position_max = 64.0;	//mm
	static constexpr double speed_min =     5.0;	//mm/s
	static constexpr double speed_max =   300.0;	//mm/s
	static constexpr double force_min =   	5.0;	//N
	static constexpr double force_max =	   20.0;    //N

	static constexpr double speed_default = 50.0;	//mm/s
	static constexpr double force_default =	10.0;  	//N

private:
	WSG25Proxy::Ptr wsg25_;
};

} /* namespace gripper */

#endif /* D7PC_ROS_CARMEN_IIWA_INCLUDE_CARMEN_IIWA_GRIPPER_WSG25GRIPPER_H_ */
