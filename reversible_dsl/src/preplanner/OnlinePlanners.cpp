///*
// * OnlinePlanners.cpp
// *
// *  Created on: Mar 7, 2017
// *      Author: josl
// */
//
//#include "OnlinePlanners.h"
//#include "../colors.hpp"
//
//namespace preplanner
//{
//
//OnlinePlanners::OnlinePlanners(std::shared_ptr<KukaIIWAPlanner> plannerTnt, PlannerInterface::Ptr plannerTfi, rw::models::WorkCell::Ptr workcell, rw::models::Device::Ptr device) :
//		plannerTnt(plannerTnt),
//		plannerTfi(plannerTfi),
//		workcell(workcell),
//		device(device)
//{
//}
//
//OnlinePlanners::~OnlinePlanners()
//{
//}
//
//Path OnlinePlanners::plan(const rw::math::Transform3D<>& baseTend, const rw::kinematics::State& state) const
//{
////	std::cout << dsl::color::BOLDGREEN << "Planner 1 START" << dsl::color::DEFAULT << std::endl;
////	double time = 4;
////	PlannerInterface::PlannerResult res = plannerTfi->plan(baseTend, state, time, PlannerInterface::plannertype::TrajOpt, PlannerInterface::samplerType::Uniform);
////
////	if(res.solved == false)
////	{
////		std::vector<rw::math::Q> empty;
////		return empty;
////	}
////
////	std::cout << dsl::color::BOLDGREEN << "Planner 1 END" << dsl::color::DEFAULT << std::endl;
////	savePath(res.path, state);
////	return res.path;
//
//	Path p;
//	p.path = plannerTnt->plan(baseTend, state);
//	p.qStart = device->getQ(state);
//	p.qEnd = p.path.back();
//	p.baseTend = baseTend;
//	return p;
//}
//
//Path OnlinePlanners::plan(const rw::math::Q& qTo, const rw::kinematics::State& state) const
//{
////	std::cout << dsl::color::BOLDGREEN << "Planner 2 START" << dsl::color::DEFAULT << std::endl;
////	double time = 4;
////	PlannerInterface::PlannerResult res = plannerTfi->plan(qTo, state, time, PlannerInterface::plannertype::TrajOpt, PlannerInterface::samplerType::Uniform);
////
////	if(res.solved == false)
////	{
////		std::vector<rw::math::Q> empty;
////		return empty;
////	}
////
////	std::cout << dsl::color::BOLDGREEN << "Planner 2 END" << dsl::color::DEFAULT << std::endl;
////	savePath(res.path, state);
////	return res.path;
//	Path p;
//	p.path = plannerTnt->plan(qTo, state);
//	p.qStart = device->getQ(state);
//	p.qEnd = p.path.back();
//	rw::kinematics::State stateEnd = state;
//	device->setQ(p.qEnd, stateEnd);
//	p.baseTend = device->baseTend(state);
//	return p;
//}
//
//} /* namespace preplanner */
