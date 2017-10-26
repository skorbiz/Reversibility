/*
 * Planner.cpp
 *
 *  Created on: Feb 16, 2017
 *      Author: josl
 */

#include <Planner.h>
#include <rw/common/Ptr.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/StateData.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

//#include <tfi_planner/planners/PlannerInterface.hpp>
//namespace planner {

//Planner::Planner(std::shared_ptr<KukaIIWAPlanner> plannerTnt, std::shared_ptr<PlannerInterface> plannerTfi, rw::models::WorkCell::Ptr workcell, rw::models::Device::Ptr device) :
//		plannerTnt(plannerTnt),
//		plannerTfi(plannerTfi),
//		pathCounter(0),
//		workcell(workcell),
//		device(device)
//{
//}

Planner::Planner(std::shared_ptr<KukaIIWAPlanner> plannerTnt, rw::models::WorkCell::Ptr workcell, rw::models::Device::Ptr device) :
		plannerTnt(plannerTnt),
		pathCounter(0),
		workcell(workcell),
		device(device)
{
}

Planner::~Planner()
{
}

std::vector<rw::math::Q> Planner::plan(const rw::math::Transform3D<>& baseTend, const rw::kinematics::State& state) const
{
//	std::cout << dsl::color::BOLDGREEN << "Planner 1 START" << dsl::color::DEFAULT << std::endl;
//	double time = 4;
//
//	PlannerInterface::PlannerResult res = plannerTfi->plan(baseTend, state, time, PlannerInterface::plannertype::TrajOpt, PlannerInterface::samplerType::Uniform);
//
//	if(res.solved == false)
//	{
//		std::vector<rw::math::Q> empty;
//		return empty;
//	}
//
//	std::cout << dsl::color::BOLDGREEN << "Planner 1 END" << dsl::color::DEFAULT << std::endl;
//	savePath(res.path, state);
//	return res.path;
	std::vector<rw::math::Q> path = plannerTnt->plan(baseTend, state);

	if(isPathCollisionFree(state, path) == false)
	{
		std::vector<rw::math::Q> empty;
		return empty;
	}
	savePath(path, state);
	return path;
}

std::vector<rw::math::Q> Planner::plan(const rw::math::Q& qTo, const rw::kinematics::State& state) const
{
//	std::cout << dsl::color::BOLDGREEN << "Planner 2 START" << dsl::color::DEFAULT << std::endl;
//	double time = 4;
//	PlannerInterface::PlannerResult res = plannerTfi->plan(qTo, state, time, PlannerInterface::plannertype::TrajOpt, PlannerInterface::samplerType::Uniform);
//
//	if(res.solved == false)
//	{
//		std::vector<rw::math::Q> empty;
//		return empty;
//	}
//
//	std::cout << dsl::color::BOLDGREEN << "Planner 2 END" << dsl::color::DEFAULT << std::endl;
//	savePath(res.path, state);
//	return res.path;
	std::vector<rw::math::Q> path = plannerTnt->plan(qTo, state);

	if(isPathCollisionFree(state, path) == false)
	{
		std::vector<rw::math::Q> empty;
		return empty;
	}

	savePath(path, state);
	return path;
}

std::vector<rw::math::Q> Planner::solve(const rw::math::Transform3D<>& baseTframe, const rw::kinematics::Frame* frame,  const rw::kinematics::State& state) const
{
//	std::cout << dsl::color::BOLDGREEN << "Planner 3 START" << dsl::color::DEFAULT << std::endl;
	std::vector<rw::math::Q> path = plannerTnt->planLinear(baseTframe, frame, state, false);
//	std::cout << dsl::color::BOLDGREEN << "Planner 3 END" << dsl::color::DEFAULT << std::endl;
	savePath(path, state);
	return path;
}


bool Planner::isPathCollisionFree(const rw::kinematics::State &state, std::vector<rw::math::Q> &path) const
{
	rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy();
	rw::proximity::CollisionDetector detector(workcell, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());
	rw::proximity::CollisionDetector::QueryResult data;
	rw::kinematics::State updated_state = state;

	bool isInCollision;

	for(rw::math::Q & q : path)
	{
		device->setQ(q, updated_state);
		bool isInCollision = detector.inCollision(updated_state, &data);
		if(isInCollision)
		{
			std::cout << __PRETTY_FUNCTION__ << "Configuration in collision: " << q << std::endl;
			std::cout << __PRETTY_FUNCTION__ << "Colliding frames: ";
			rw::kinematics::FramePairSet fps = data.collidingFrames;
			for (rw::kinematics::FramePairSet::iterator it = fps.begin(); it != fps.end(); it++)
				std::cout << (*it).first->getName() << " " << (*it).second->getName() << "   ";
			std::cout << std::endl;
			return false;
		}
	}
	return true;
}

void Planner::savePath(const rw::trajectory::QPath& path, const rw::kinematics::State& state) const
{
	if(path.size() == 0)
		return;
	rw::trajectory::TimedStatePath tStatePath = rw::trajectory::TimedUtil::makeTimedStatePath(
	            *workcell,
	           rw::models::Models::getStatePath(*device, path, state));
	std::string saveName = "path" + std::to_string(pathCounter) + ".rwplay";
	rw::loaders::PathLoader::storeTimedStatePath(*workcell, tStatePath, saveName);
	pathCounter++;
}




//} /* namespace planner */
