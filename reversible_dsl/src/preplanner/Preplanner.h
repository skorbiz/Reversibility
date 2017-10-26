///*
// * Preplanner.h
// *
// *  Created on: Mar 7, 2017
// *      Author: josl
// */
//
//#ifndef D7PC_ROS_REVERSIBLE_DSL_SRC_PREPLANNER_PREPLANNER_H_
//#define D7PC_ROS_REVERSIBLE_DSL_SRC_PREPLANNER_PREPLANNER_H_
//
//#include <rw/math/Q.hpp>
//#include <rw/math/Transform3D.hpp>
//
//#include "Path.h"
//#include "../D7cpWorkcell.h"
//
//
//namespace preplanner
//{
//
//class Preplanner
//{
//public:
//	Preplanner();
//	Preplanner(D7cpWorkcell wc);
//	virtual ~Preplanner();
//
////	std::vector<rw::math::Q> plan(const rw::math::Transform3D<>& baseTend, const rw::kinematics::State& state) const;
////	std::vector<rw::math::Q> plan(const rw::math::Q& qTo, const rw::kinematics::State& state) const;
//
//private:
//	rw::math::Q qDispenser;
//	rw::math::Q qSpring;
//	rw::math::Q qThermoElement;
//	rw::math::Q qFixture;
//	rw::math::Q qInspectionOutershell;
//	rw::math::Q qInspectionInnerStructure;
//	rw::math::Q qInspectionPin;
//	rw::math::Q qHome;
//
//	//Anyfeeder
//	static const int lengthNanyfeeder = 8;
//	static const int widthNanyfeeder = 4;
//	static const int rotationNanyfeeder = 8;
//	rw::math::Transform3D<> baseTanyfeeder[lengthNanyfeeder][widthNanyfeeder][rotationNanyfeeder];
//	rw::math::Transform3D<> anyfeederTtool;
//
//	//Pin-feeder
//	static const int lengthNpin = 6;
//	static const int widthNpin = 3;
//	rw::math::Q qPin[lengthNpin][widthNpin];
//
//	//Fixture
//	static const int rotationNfixture = 16;
//	rw::math::Transform3D<> baseTfixture[rotationNfixture];
//	rw::math::Transform3D<> fixtureTtool;
//
//	//Paths
//	Path dispenser2anyfeeder[lengthNanyfeeder][widthNanyfeeder][rotationNanyfeeder];
//	Path dispenser2inspectionOutershell;
//	Path dispenser2inspectionInnerstructure;
//
//	Path home2inspectionOutershell;
//	Path home2inspectionInnerstructure;
//	Path home2inspectionPin;
//
//	Path home2pin[lengthNpin][widthNpin];
//	Path home2spring;
//	Path home2thermoelement;
//
//	Path home2fixture[rotationNfixture];
//
//	std::vector<Path> paths_all;
//
//	rw::models::Device::Ptr device;
//
//
//
//
//
//};
//
//} /* namespace preplanner */
//
//#endif /* D7PC_ROS_REVERSIBLE_DSL_SRC_PREPLANNER_PREPLANNER_H_ */
