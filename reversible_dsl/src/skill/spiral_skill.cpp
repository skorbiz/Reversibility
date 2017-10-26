#include "spiral_skill.hpp"

#include <d7cp_proxys/robot_movement_interface/RobworkInterface.h>

#include <rw/trajectory.hpp>

#include <math.h>
#include <vector>

using namespace std;

constexpr SpiralSkill::parameters SpiralSkill::d7pc_pin_parameters;
constexpr SpiralSkill::parameters SpiralSkill::sim_optimized_d7pc_parameters;


SpiralSkill::SpiralSkill(std::shared_ptr<robot_movement_interface::RobworkInterface> controller,
		std::shared_ptr<D7cpWorkcell> wc, rw::math::Transform3D<> toolTobject) : mController(controller), mWc(wc)
{
	rw::math::Transform3D<> objectTmalePin = mWc->pinGraspFrame->fTf(mWc->pinInsertionFrame.get(), mWc->getDefaultState());
	mmaleTtool = rw::math::inverse(toolTobject*objectTmalePin);
}

bool SpiralSkill::run_spiral_skill(parameters param) {
	std::vector<rw::math::Transform3D<> >  baseTtool_traj =  generateTrajectory_baseTtool(param);


	rw::math::Transform3D<> temp;
	for(rw::math::Transform3D<> & baseTtool : baseTtool_traj) {

		mController->moveComplience(baseTtool);
		temp = baseTtool;

		while(mController->is_moving()) {
			if( is_pin_in_hole( mController->getBaseTtool() ) ) {
				mController->stop();
				mController->moveComplience(baseTtool_traj[1]);
				while(mController->is_moving());
				return true;
			}
		}
	}
	return false;
}

std::vector<rw::math::Transform3D<> > SpiralSkill::generateTrajectory_baseTtool(parameters param) const{

	std::vector<rw::math::Transform3D<> > trajectory = generateTrajectory(param);
	std::shared_ptr<rw::kinematics::Frame> innerStructure = mWc->innerStructureAssemblyFrame;

	//	Assembly_InnerStructure_female
	std::shared_ptr<rw::kinematics::Frame> female = mWc->innerStructureFemaleAssemblyFrame;
	rw::math::Transform3D<> baseTfemale = mWc->baseFrame->fTf(female.get(), mWc->getDefaultState());

	for(rw::math::Transform3D<> & t3d : trajectory ) // "&" extreamly important else each element is copied -> updated -> deleted.
	{
		t3d = baseTfemale * t3d * mmaleTtool;
	}

	return trajectory;
}


std::vector<rw::math::Transform3D<> > SpiralSkill::generateTrajectory(parameters param) const{

	std::vector<rw::math::Transform3D<> > T3Dpath;

	auto initial_pose = rw::math::Transform3D<>(rw::math::Vector3D<>(0,0,param.length_start_traj));
	T3Dpath.push_back(initial_pose);

	for(double t = 0; t <  2 * rw::math::Pi * param.n; t += param.d_path){
//		double val = 1.0*t*param.r / ( 2 * rw::math::Pi );
//		double x = sin(t) * param.sx * val;
//		double y = cos(t) * param.sy * val;
//		auto spinal_pose  = rw::math::Transform3D<>(rw::math::Vector3D<>(x,y, -param.length_push ));
        
		double a = param.r;
		double d = 0.035;

        rw::math::Vector3D<> peg_tip_pose(
				-a * t * cos(t),
				-a * t * sin(t),
				sqrt(d*d -a*a *t*t) - param.length_push - d
		);
		
		double Y = asin(a * t * cos(t) / d);
		double X = atan2( -a * t * sin(t) / (d * cos(Y)),sqrt(d*d - a*a *t*t) / (d * cos(Y)) );

		rw::math::Rotation3D<> peg_angle(
				cos(Y), 0, sin(Y),
				sin(X) * sin(Y), cos(X), -cos(Y) * sin(X),
				-cos(X) * sin(Y), sin(X), cos(X) * cos(Y)
		);
		rw::math::Transform3D<> peg_tip(peg_tip_pose,peg_angle);

		//TEMP PRINT
		rw::math::RPY<> rpy(peg_angle);
		std::cout << peg_tip.P() << " RPY: " << rpy[0] << " " << rpy[1] << " " << rpy[2] << std::endl;

        
//		T3Dpath.push_back(spinal_pose);
        T3Dpath.push_back(peg_tip);
	}

	return T3Dpath;
}

bool SpiralSkill::is_pin_in_hole(rw::math::Transform3D<> baseTtool) const
{
//	std::cout << baseTtool.P()[2] << std::endl;
	if(baseTtool.P()[2] < 0.1108) //0.1105
		return true;
	return false;
}
