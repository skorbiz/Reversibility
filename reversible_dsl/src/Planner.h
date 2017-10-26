/*
 * Planner.h
 *
 *  Created on: Feb 16, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DSL_SRC_PLANNER_PLANNER_H_
#define D7PC_ROS_REVERSIBLE_DSL_SRC_PLANNER_PLANNER_H_

#include <memory.h>

#include "KukaIIWAPlanner.hpp"
#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/trajectory/Path.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rw/models/Models.hpp>
#include <rw/trajectory/TimedUtil.hpp>


//namespace planner {
//class PlannerInterface;

class Planner {
public:
//	Planner(std::shared_ptr<KukaIIWAPlanner> plannerTnt, std::shared_ptr<PlannerInterface> plannerTfi, rw::models::WorkCell::Ptr workcell, rw::models::Device::Ptr device);
	Planner(std::shared_ptr<KukaIIWAPlanner> plannerTnt, rw::models::WorkCell::Ptr workcell, rw::models::Device::Ptr device);

	virtual ~Planner();

	std::vector<rw::math::Q> plan(const rw::math::Transform3D<>& baseTend, const rw::kinematics::State& state) const;
	std::vector<rw::math::Q> plan(const rw::math::Q& qTo, const rw::kinematics::State& state) const;
	std::vector<rw::math::Q> solve(const rw::math::Transform3D<>& baseTframe, const rw::kinematics::Frame* frame,  const rw::kinematics::State& state) const;

	bool isPathCollisionFree(const rw::kinematics::State &state, std::vector<rw::math::Q> &path) const;

private:
	void savePath(const rw::trajectory::QPath& path, const rw::kinematics::State& state) const;

private:
	std::shared_ptr<KukaIIWAPlanner> plannerTnt;
//	std::shared_ptr<PlannerInterface> plannerTfi;

    mutable int pathCounter;
	rw::models::WorkCell::Ptr workcell;
    rw::models::Device::Ptr device;

};

//} /* namespace planner */

#endif /* D7PC_ROS_REVERSIBLE_DSL_SRC_PLANNER_PLANNER_H_ */
