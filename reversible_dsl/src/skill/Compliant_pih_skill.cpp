/*
 * compliant_pih_skill.cpp
 *
 *  Created on: Mar 20, 2017
 *      Author: jpb
 */

#include "Compliant_pih_skill.h"

using namespace rw::math;

constexpr Compliant_pih_skill::parameters Compliant_pih_skill::d7pc_innerstructure;


Compliant_pih_skill::Compliant_pih_skill(
		std::shared_ptr<robot_movement_interface::RobworkInterface> robot,
		std::shared_ptr<D7cpWorkcell> wc, Transform3D<> toolTobject) :
		mRobot(robot), mWc(wc)
{
	rw::math::Transform3D<> objectTmalePin = mWc->innerStructureGraspFrame->fTf(mWc->innerStructureTipInGripper.get(), mWc->getDefaultState());
	mmaleTtool = rw::math::inverse(toolTobject*objectTmalePin);

}

void Compliant_pih_skill::run_skill(const parameters & param) {
	std::vector<rw::math::Transform3D<> >  baseTtool_traj =  generateTrajectory_baseTtool(param);

		int i = 0;
		for(rw::math::Transform3D<> & baseTtool : baseTtool_traj) {
			mRobot->moveComplience(baseTtool);
			while(mRobot->is_moving());
			std::cout << i++ << std::endl;
		}
}

std::vector<Transform3D<> > Compliant_pih_skill::generateTrajectory_baseTtool(const parameters & param) {
	std::vector<Transform3D<> > trajectory = generateTrajectory(param);

	std::shared_ptr<rw::kinematics::Frame> base = mWc->baseFrame;
	std::shared_ptr<rw::kinematics::Frame> outerShell_InnerRing = mWc->outershellInnerRing;
	Transform3D<> baseTouterShellInnerRing = base->fTf(outerShell_InnerRing.get(), mWc->getDefaultState());


//	std::shared_ptr<rw::kinematics::Frame> innerStructure_tip = mWc->innerStructureTipInGripper;
//	std::shared_ptr<rw::kinematics::Frame> tool = mWc->toolFrame;
//	Transform3D<> innerTipTtool = innerStructure_tip->fTf( tool.get(), mWc->getDefaultState());

	for(rw::math::Transform3D<> & t3d : trajectory )
	{
		t3d = baseTouterShellInnerRing * t3d * mmaleTtool;
	}
	return trajectory;
}

std::vector<rw::math::Transform3D<> > Compliant_pih_skill::generateTrajectory(parameters param) {
	//tip of inner structure to center of hole circle.
	std::vector<rw::math::Transform3D<> > T3Dpath;

	Transform3D<> initial_pose(Vector3D<>( 0,- param.length_start_traj, 0 ));
	T3Dpath.push_back(initial_pose);

	Transform3D<> top_pose(Vector3D<>( 0, - param.length_top_rot, 0 ));
	T3Dpath.push_back(top_pose);

	Transform3D<> angled_top_pose = rotate_around_point(top_pose,Vector3D<>( 0, - param.length_rot_end , 0), RPY<>(param.tilt_angle, 0, 0 ) );
	T3Dpath.push_back(angled_top_pose);

	Transform3D<> bot_pose(Vector3D<>( 0, param.length_push, 0 ));

	Transform3D<> angled_pose = rotate_around_point(bot_pose,Vector3D<>( 0, - param.length_rot_end , 0), RPY<>(param.tilt_angle, 0, 0 ) );
	(Vector3D<>( 0, param.length_push, 0),RPY<>(param.tilt_angle, 0, 0 ));
	T3Dpath.push_back(angled_pose);
	T3Dpath.push_back(bot_pose);

//	Transform3D<> end_pose(Vector3D<>( 0, param.length_insertion_traj, 0));
//	T3Dpath.push_back(end_pose);

	return T3Dpath;
}

rw::math::Transform3D<> Compliant_pih_skill::rotate_around_point(rw::math::Transform3D<> T, rw::math::Vector3D<> rot_point, rw::math::RPY<> angle) {
	Transform3D<> tTpoint(rot_point);
	Transform3D<> pointTt( - rot_point);
	Transform3D<> rot(angle.toRotation3D());

	return T* tTpoint * rot * pointTt;
}
