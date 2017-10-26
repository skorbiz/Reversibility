/*
 * D7PCGripper.h
 *
 *  Created on: Feb 7, 2017
 *      Author: jpb
 */

#ifndef D7PC_ROS_CARMEN_IIWA_INCLUDE_CARMEN_IIWA_GRIPPER_D7PCGRIPPER_H_
#define D7PC_ROS_CARMEN_IIWA_INCLUDE_CARMEN_IIWA_GRIPPER_D7PCGRIPPER_H_

#include "WSG25Gripper.h"

namespace gripper {

class D7PC_Gripper {
public:

	enum class object { inner_structure, outer_shell, thermo_element, pin, spring, skrew_part, none };

	struct GraspConfig {
		double speed;
		double clerance;
		double gripping_force;
		double grasp_width;
		object obj;
	};

	typedef std::shared_ptr<D7PC_Gripper> Ptr;
	D7PC_Gripper(WSG25Gripper::Ptr gripper);
	virtual ~D7PC_Gripper() {}

	void open(GraspConfig config);
	void close(GraspConfig config);

	bool in_hand(GraspConfig config);
	bool grasped_on_tab_inner_structure();

	static constexpr GraspConfig inner_structure 	= { .speed = 50, .clerance = 20, .gripping_force = 25, .grasp_width = 29.0, .obj = object::inner_structure };
	static constexpr GraspConfig outer_shell 		= { .speed = 50, .clerance = 20, .gripping_force = 25, .grasp_width = 34.0, .obj = object::outer_shell };
	static constexpr GraspConfig thermo_element 	= { .speed = 50, .clerance = 15, .gripping_force = 25, .grasp_width = 12.5, .obj = object::thermo_element };
	static constexpr GraspConfig pin 				= { .speed = 50, .clerance = 15, .gripping_force = 25, .grasp_width =  5.0, .obj = object::pin };
	static constexpr GraspConfig spring			 	= { .speed = 50, .clerance = 25, .gripping_force = 10, .grasp_width =  5.0, .obj = object::spring };
	static constexpr GraspConfig skrew_part		 	= { .speed = 50, .clerance = 25, .gripping_force = 10, .grasp_width = 26.7, .obj = object::skrew_part };

private:
	WSG25Gripper::Ptr gripper;
	object object_in_hand;
};


} /* namespace gripper */

#endif /* D7PC_ROS_CARMEN_IIWA_INCLUDE_CARMEN_IIWA_GRIPPER_D7PCGRIPPER_H_ */
