///*
// * Preplanner.cpp
// *
// *  Created on: Mar 7, 2017
// *      Author: josl
// */
//
//#include "Preplanner.h"
//#include "OnlinePlanners.h"
//
//namespace preplanner {
//
//Preplanner::Preplanner()
//{
//}
//
//Preplanner::~Preplanner()
//{
//}
//
//
//Preplanner::Preplanner(D7cpWorkcell wc)
//{
//
//	device = wc.device;
//
//	rw::math::Q qDispenser = wc.qDispenser;
//	rw::math::Q qSpring = wc.qSpringPickInit;
//	rw::math::Q qThermoElement = wc.qThermoelementPickInit;
//	rw::math::Q qInspectionOutershell = wc.qInspectOuterShell;
//	rw::math::Q qInspectionInnerStructure = wc.qInspectInnerStructure;
//	rw::math::Q qInspectionPin = wc.qInspectPin;
//	rw::math::Q qHome = wc.qZero;
//	//rw::math::Q qFixture = ;
//
//	//Anyfeeder
//	rw::kinematics::State state = wc.getDefaultState();
//	rw::math::Transform3D<> base2init = wc.converter->frameTframe(wc.baseFrame,wc.pathplannerAnyFeederInit,state);
//	for(int l = 0; l < lengthNanyfeeder; l++)
//		for(int w = 0; w < widthNanyfeeder; w++)
//			for(int r = 0; r < rotationNanyfeeder; r++)
//			{
//				double dx = 0.20/widthNanyfeeder;
//				double dy = -0.35/lengthNanyfeeder;
//				double dr = (2.0*M_PI)/rotationNanyfeeder;
//				rw::math::Vector3D<> vec(dx*w, dy*l, 0);
//				rw::math::RPY<> rot(dr*r, 0, 0);
//				rw::math::Transform3D<> init2sample(vec, rot.toRotation3D());
//				baseTanyfeeder[l][w][r] =  base2init * init2sample;
//			}
//
//	rw::math::RPY<> anyfeederTtoolRot(0,0,-M_PI/2.0);
//	anyfeederTtool = rw::math::Transform3D<>(anyfeederTtoolRot.toRotation3D());
//
//
//	//Pin-feeder
//	static const int lengthNpin = 6;
//	static const int widthNpin = 3;
//	assert(lengthNpin == wc.Nj);
//	assert(widthNpin == wc.Ni);
//	for (unsigned int i = 0; i < widthNpin; i++)
//		for (unsigned int j = 0; j < lengthNpin; j++)
//			qPin[j][i] = wc.qPinHole[i][j];
//
//	//Fixture
//	rw::math::Transform3D<> base2fixtureInit = wc.converter->frameTframe(wc.baseFrame, wc.outershellAssemblyRetractedFrame, state);
//	for(int r = 0; r < rotationNfixture; r++)
//	{
//		double dr = (2.0*M_PI)/rotationNfixture;
//		rw::math::RPY<> rot(0, dr*r, 0);
//		rw::math::Transform3D<> fixture2sample(rot.toRotation3D());
//		baseTfixture[r] =  base2fixtureInit * fixture2sample;
//	}
//	rw::math::RPY<> fixtureTtoolRot(M_PI,0,0);
//	fixtureTtool = rw::math::Transform3D<>(fixtureTtoolRot.toRotation3D());
//
//
//	//Paths ///////////////////////////////
//	rw::math::Transform3D<> toolTend = rw::kinematics::Kinematics::frameTframe(wc.toolFrame.get(), wc.device->getEnd(), state);
//	std::shared_ptr<OnlinePlanners> planner = std::make_shared<OnlinePlanners>(wc.plannerTnt, wc.plannerTfi, wc.wc, wc.device);
//
//	//dispenser2anyfeeder
//	int totalAnyfeederPaths = lengthNanyfeeder * widthNanyfeeder * rotationNanyfeeder;
//	int calculatedAnyfeederPaths = 0;
//	for(int l = 0; l < lengthNanyfeeder; l++)
//		for(int w = 0; w < widthNanyfeeder; w++)
//			for(int r = 0; r < rotationNanyfeeder; r++)
//			{
//				rw::kinematics::State state = wc.getDefaultState();
//				wc.attachTo(wc.innerStructureFrame, wc.innerStructureGraspFrame, state);
//				wc.setQ(qDispenser, state);
//				dispenser2anyfeeder[l][w][r] = planner->plan(baseTanyfeeder[l][w][r]*anyfeederTtool*toolTend,state);
//				std::cout << calculatedAnyfeederPaths++ << " / " << totalAnyfeederPaths << std::endl;
//			}
//
//	//dispenser2inspectionOutershell
//	{
//		rw::kinematics::State state = wc.getDefaultState();
//		wc.attachTo(wc.outershellFrame, wc.outershellGraspFrame, state);
//		wc.setQ(qDispenser, state);
//		dispenser2inspectionOutershell = planner->plan(qInspectionOutershell,state);;
//
//	}
//
//	//dispenser2inspectionInnerstructure
//	{
//		rw::kinematics::State state = wc.getDefaultState();
//		wc.attachTo(wc.innerStructureFrame, wc.innerStructureGraspFrame, state);
//		wc.setQ(qDispenser, state);
//		dispenser2inspectionInnerstructure = planner->plan(qInspectionOutershell,state);
//	}
//
//	//home2inspectionOutershell
//	{
//		rw::kinematics::State state = wc.getDefaultState();
//		wc.attachTo(wc.outershellFrame, wc.outershellGraspFrame, state);
//		wc.setQ(qHome, state);
//		home2inspectionOutershell = planner->plan(qInspectionOutershell,state);
//	}
//
//	//home2inspectionInnerstructure
//	{
//		rw::kinematics::State state = wc.getDefaultState();
//		wc.attachTo(wc.innerStructureFrame, wc.innerStructureGraspFrame, state);
//		wc.setQ(qHome, state);
//		home2inspectionInnerstructure = planner->plan(qInspectionInnerStructure,state);
//	}
//
//	//home2inspectionPin
//	{
//		rw::kinematics::State state = wc.getDefaultState();
//		wc.attachTo(wc.pinFrame, wc.pinGraspFrame, state);
//		wc.setQ(qHome, state);
//		home2inspectionPin = planner->plan(qInspectionPin,state);
//	}
//
//	//home2spring
//	{
//		rw::kinematics::State state = wc.getDefaultState();
//		wc.attachTo(wc.springFrame, wc.springGraspFrame, state);
//		wc.setQ(qHome, state);
//		home2spring  = planner->plan(qSpring,state);
//	}
//
//	//home2inspectionPin
//	{
//		rw::kinematics::State state = wc.getDefaultState();
//		wc.attachTo(wc.thermoelementFrame, wc.thermoelementGraspFrame, state);
//		wc.setQ(qHome, state);
//		home2thermoelement = planner->plan(qThermoElement,state);
//	}
//
//	//Home2pin
//	for (unsigned int i = 0; i < widthNpin; i++)
//		for (unsigned int j = 0; j < lengthNpin; j++)
//		{
//			rw::kinematics::State state = wc.getDefaultState();
//			wc.attachTo(wc.pinFrame, wc.pinGraspFrame, state);
//			wc.setQ(qHome, state);
//			home2pin[j][i] = planner->plan(qPin[j][i],state);
//		}
//
//	//home2fixture
//	for(int r = 0; r < rotationNfixture; r++)
//	{
//		rw::kinematics::State state = wc.getDefaultState();
//		wc.attachTo(wc.innerStructureFrame, wc.innerStructureGraspFrame, state);
//		wc.setQ(qHome, state);
//		home2fixture[r] = planner->plan(baseTfixture[r]*fixtureTtool*toolTend, state);
//	}
//
//
//	//Paths collect
//	for(int l = 0; l < lengthNanyfeeder; l++)
//		for(int w = 0; w < widthNanyfeeder; w++)
//			for(int r = 0; r < rotationNanyfeeder; r++)
//				paths_all.push_back(dispenser2anyfeeder[l][w][r]);
//
//	paths_all.push_back(dispenser2inspectionOutershell);
//	paths_all.push_back( dispenser2inspectionInnerstructure);
//
//	paths_all.push_back( home2inspectionOutershell);
//	paths_all.push_back( home2inspectionInnerstructure);
//	paths_all.push_back( home2inspectionPin);
//
//	for (unsigned int i = 0; i < widthNpin; i++)
//		for (unsigned int j = 0; j < lengthNpin; j++)
//			paths_all.push_back( home2pin[j][i]);
//
//	paths_all.push_back( home2spring);
//	paths_all.push_back( home2thermoelement);
//
//	for(int r = 0; r < rotationNfixture; r++)
//		paths_all.push_back( home2fixture[r]);
//
//}
//
//
////std::vector<rw::math::Q> Preplanner::plan(const rw::math::Q& qTo, const rw::kinematics::State& state) const
////{
////	assert(paths_all.size() > 0);
////	std::vector<Path> paths = paths_all;
////
////	rw::math::Q qStart = device->getQ(state);
////
////	//Discard principles
////	for(size_t i = 0; i < paths.size(); i++)
////		if((qStart-qTo).norm2() > 0.01)
////			paths.erase(paths.begin() + i);
////
////	assert(paths_all.size() > 0);
////	Path best_match = paths[0];
////
////
////
////}
//
//
//
//} /* namespace preplanner */
