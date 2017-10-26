/*
 * D7cpObject.h
 *
 *  Created on: Feb 9, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DSL_SRC_D7CPOBJECT_H_
#define D7PC_ROS_REVERSIBLE_DSL_SRC_D7CPOBJECT_H_

#include <memory.h>
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>

class D7cpObject
{

public:
	D7cpObject(int part_number, std::string part_name, const rw::models::WorkCell::Ptr wc);
	D7cpObject(std::shared_ptr<rw::kinematics::Frame> obj_frame,
			std::shared_ptr<rw::kinematics::Frame> obj_grasp_frame,
			std::shared_ptr<rw::kinematics::Frame> obj_detach_frame,
			std::shared_ptr<rw::kinematics::Frame> obj_assembly_frame);
	virtual ~D7cpObject();

    void grasp(rw::kinematics::State& state) const;
    void detach(rw::kinematics::State& state) const;
//    void assembly(rw::kinematics::State& state) const;

private:
    std::shared_ptr<rw::kinematics::Frame> objFrame;
    std::shared_ptr<rw::kinematics::Frame> graspFrame;
    std::shared_ptr<rw::kinematics::Frame> detachFrame;
    std::shared_ptr<rw::kinematics::Frame> assemblyFrame;


};

#endif /* D7PC_ROS_REVERSIBLE_DSL_SRC_D7CPOBJECT_H_ */
