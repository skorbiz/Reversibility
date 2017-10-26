/*
 * D7cpObject.cpp
 *
 *  Created on: Feb 9, 2017
 *      Author: josl
 */

#include "D7cpObject.h"

D7cpObject::D7cpObject(int part_number, std::string part_name, const rw::models::WorkCell::Ptr wc)
{
	std::string obj = "Part" + std::to_string(part_number) + "_" + part_name;
	std::string grasp = "Grasp_" + part_name;
	std::string detach = "Part" + std::to_string(part_number) + "_Detach";
	std::string assembly = "Assembly_" + part_name;

	std::shared_ptr<rw::kinematics::Frame> objFramePtr(wc->findFrame(obj));
	std::shared_ptr<rw::kinematics::Frame> graspFramePtr(wc->findFrame(grasp));
	std::shared_ptr<rw::kinematics::Frame> detachFramePtr(wc->findFrame(detach));
	std::shared_ptr<rw::kinematics::Frame> assemblyFramePtr(wc->findFrame(assembly));

	objFrame = objFramePtr;
	graspFrame = graspFramePtr;
	detachFrame = detachFramePtr;
	assemblyFrame = assemblyFramePtr;

	if(objFrame == NULL) std::cerr << "Could not find the frame " << obj << std::endl;
	if(graspFrame == NULL) std::cerr << "Could not find the frame " << grasp << std::endl;
	if(detachFrame == NULL) std::cerr << "Could not find the frame " << detach << std::endl;
	if(assemblyFrame == NULL) std::cerr << "Could not find the frame " << assembly << std::endl;

}

D7cpObject::D7cpObject(
		std::shared_ptr<rw::kinematics::Frame> obj_frame,
		std::shared_ptr<rw::kinematics::Frame> obj_grasp_frame,
		std::shared_ptr<rw::kinematics::Frame> obj_detach_frame,
		std::shared_ptr<rw::kinematics::Frame> obj_assembly_frame) :
	objFrame(obj_frame),
	graspFrame(obj_grasp_frame),
	detachFrame(obj_detach_frame),
	assemblyFrame(obj_assembly_frame)
{
}

D7cpObject::~D7cpObject()
{
}

void D7cpObject::grasp(rw::kinematics::State& state) const
{
	objFrame.get()->attachTo(graspFrame.get(), state);
}

void D7cpObject::detach(rw::kinematics::State& state) const
{
	objFrame.get()->attachTo(detachFrame.get(), state);
}

//void D7cpObject::assembly(rw::kinematics::State& state) const
//{
//	objFrame.get()->attachTo(assemblyFrame.get(), state);
//}
