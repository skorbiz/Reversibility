/*
 * wiggleskill.cpp
 *
 *  Created on: Mar 20, 2017
 *      Author: jpb
 */

#include "wiggle_skill.h"

#include <rw/math/Math.hpp>

using namespace rw::math;

WiggleSkill::WiggleSkill(
		std::shared_ptr<robot_movement_interface::RobworkInterface> controller,
		std::shared_ptr<D7cpWorkcell> wc,
		Transform3D<> toolTobject) :
				mController(controller), mWc(wc)
{
	Transform3D<> objectTmalePin = mWc->pinGraspFrame->fTf(mWc->pinInsertionFrame.get(), mWc->getDefaultState());
	mmaleTtool = inverse(toolTobject*objectTmalePin);
}

WiggleSkill::parameters WiggleSkill::defaultParam() {
	parameters p;
	p.outer_angle = 12 * rw::math::Pi / 180;
	p.push_depth = 0.005; //0.002
	p.wiggles_count = 2;
	p.depth_for_success =  0.1055; //0.1055
	return p;
}

bool WiggleSkill::run_skill(const parameters & param) {

	std::vector<rw::math::Transform3D<> >  baseTtool_traj =  generateTrajectory_baseTtool(param);
	assert(baseTtool_traj.size() > 1);


	mController->moveLin(baseTtool_traj.front());

	for(rw::math::Transform3D<> & baseTtool : baseTtool_traj) {
		mController->moveComplience(baseTtool);
		while(mController->is_moving()) {
			if( successCriteria(param, mController->getBaseTtool() ) ) {
				mController->stop();
				mController->moveComplience(baseTtool_traj[1]);
				while(mController->is_moving());

				return true;
			}
		}
	}

	mController->moveComplience(baseTtool_traj[1]);
	while(mController->is_moving());
	return false;
}

std::vector<Transform3D<> > WiggleSkill::generateTrajectory_baseTtool(const parameters & param) {
	std::vector<Transform3D<> > trajectory = generateTrajectory(param);
	std::shared_ptr<rw::kinematics::Frame> innerStructure = mWc->innerStructureAssemblyFrame;

	//	Assembly_InnerStructure_female
	std::shared_ptr<rw::kinematics::Frame> female = mWc->innerStructureFemaleAssemblyFrame;
	rw::math::Transform3D<> baseTfemale = mWc->baseFrame->fTf(female.get(), mWc->getDefaultState());

	for(rw::math::Transform3D<> & t3d : trajectory )
	{
		t3d = baseTfemale * t3d * mmaleTtool;
	}

	return trajectory;
}

std::vector<Transform3D<> > WiggleSkill::generateTrajectory(const parameters & param) {
	std::vector<rw::math::Transform3D<> > T3Dpath;

	Transform3D<> initial_pose_1(Vector3D<>(0,-0.0015,0));
	Transform3D<> initial_pose_2(Vector3D<>(0,-0.0015,-param.push_depth));
	T3Dpath.push_back(initial_pose_1);
	T3Dpath.push_back(initial_pose_2);

	for(int i = 1; i <= param.wiggles_count; i++ ) {
		double angle = ( (1.0*i) / param.wiggles_count) * param.outer_angle;

		Vector3D<> pose(0,-0.0015,-param.push_depth);
		RPY<> orientation_pos(angle,0,0);
		RPY<> orientation_neg(-angle,0,0);

		T3Dpath.push_back(Transform3D<>(pose,orientation_pos));
		T3Dpath.push_back(Transform3D<>(pose,orientation_neg));
	}

	Vector3D<> end_pose(0,0,-param.push_depth);
	T3Dpath.push_back(Transform3D<>(end_pose));


	return T3Dpath;
}

bool WiggleSkill::successCriteria(const parameters & param, rw::math::Transform3D<> baseTtool) {
	std::cout << baseTtool.P()[2] << std::endl;
    if(baseTtool.P()[2]  <  param.depth_for_success )
	    return true;
	return false;
}
