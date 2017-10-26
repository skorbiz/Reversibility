///*
// * OnlinePlanners.h
// *
// *  Created on: Mar 7, 2017
// *      Author: josl
// */
//
//#ifndef D7PC_ROS_REVERSIBLE_DSL_SRC_PREPLANNER_ONLINEPLANNERS_H_
//#define D7PC_ROS_REVERSIBLE_DSL_SRC_PREPLANNER_ONLINEPLANNERS_H_
//
//#include <memory.h>
//#include "../planner/src/planners/PlannerInterface.hpp"
//#include "../KukaIIWAPlanner.hpp"
//#include "Path.h"
//
//namespace preplanner
//{
//
//class OnlinePlanners
//{
//
//public:
//	OnlinePlanners(std::shared_ptr<KukaIIWAPlanner> plannerTnt, PlannerInterface::Ptr plannerTfi, rw::models::WorkCell::Ptr workcell, rw::models::Device::Ptr device);
//	virtual ~OnlinePlanners();
//
//	Path plan(const rw::math::Transform3D<>& baseTend, const rw::kinematics::State& state) const;
//	Path plan(const rw::math::Q& qTo, const rw::kinematics::State& state) const;
//
//	std::shared_ptr<KukaIIWAPlanner> plannerTnt;
//    PlannerInterface::Ptr plannerTfi;
//    rw::models::WorkCell::Ptr workcell;
//    rw::models::Device::Ptr device;
//
//};
//
//} /* namespace preplanner */
//
//#endif /* D7PC_ROS_REVERSIBLE_DSL_SRC_PREPLANNER_ONLINEPLANNERS_H_ */
//
