/*
a * D7PCGripper.cpp
 *
 *  Created on: Feb 7, 2017
 *      Author: jpb
 */

#include <gripper/D7PCGripper.h>

namespace gripper {

constexpr D7PC_Gripper::GraspConfig D7PC_Gripper::inner_structure;
constexpr D7PC_Gripper::GraspConfig D7PC_Gripper::outer_shell;
constexpr D7PC_Gripper::GraspConfig D7PC_Gripper::thermo_element;
constexpr D7PC_Gripper::GraspConfig D7PC_Gripper::pin;
constexpr D7PC_Gripper::GraspConfig D7PC_Gripper::spring;
constexpr D7PC_Gripper::GraspConfig D7PC_Gripper::skrew_part;

D7PC_Gripper::D7PC_Gripper(WSG25Gripper::Ptr wsg25gripper) :
		gripper(wsg25gripper),
		object_in_hand(object::none) {
}

void D7PC_Gripper::open(GraspConfig config) {
	gripper->move(config.grasp_width + config.clerance, config.speed);
	gripper->block_while_moving();
}
void D7PC_Gripper::close(GraspConfig config) {
	gripper->set_force(config.gripping_force);
	gripper->move(2, config.speed);
	gripper->block_while_moving();
	object_in_hand = config.obj;
}


bool D7PC_Gripper::in_hand(GraspConfig config) {

	if(gripper->getWidth() < config.grasp_width)
		return false;
	return true;

//	// object lost
//	if(gripper->getForce() < config.gripping_force / 2.) {
//		object_in_hand = object::none;
//		return false;
//	}
//	return true;
}

bool D7PC_Gripper::grasped_on_tab_inner_structure() {
	return object_in_hand != object::inner_structure && gripper->getWidth() > 31.0;
}

} /* namespace gripper */
