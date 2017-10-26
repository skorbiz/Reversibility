/********************************************************************************
 * Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#ifndef CAROS_KUKABROKER_DEMO_KUKAIIWAPLANNER_HPP_
#define CAROS_KUKABROKER_DEMO_KUKAIIWAPLANNER_HPP_

#include <rw/common/Ptr.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Q.hpp>

namespace rw { namespace invkin { class ClosedFormIKSolverKukaIIWA; } }
namespace rw { namespace invkin { class InvKinSolver; } }
namespace rw { namespace kinematics { class Frame; } }
namespace rw { namespace kinematics { class State; } }
namespace rw { namespace models { class Device; } }
namespace rw { namespace models { class SerialDevice; } }
namespace rw { namespace models { class WorkCell; } }
namespace rw { namespace pathplanning { class PlannerConstraint; } }
namespace rw { namespace pathplanning { class QToQPlanner; } }
namespace rw { namespace proximity { class CollisionDetector; } }
namespace rwlibs { namespace pathoptimization { class PathLengthOptimizer; } }

/**
 * @brief Planner for Kuka IIWA robot.
 */
class KukaIIWAPlanner {
public:
	/**
	 * @brief Construct new planner for a Kuka IIWA SerialDevice with name \b deviceName such that no collisions occur in the workcell \b wc .
	 * @param wc [in] model of the workcell with the device and environment.
	 * @param deviceName [in] the name of the SerialDevice in the workcell which models a 7 DOF Kuka IIWA.
	 * @param fixateJoint [in] specify a number between 0 and 6 to fixate a joint during linear movements.
	 */
	KukaIIWAPlanner(rw::common::Ptr<rw::models::WorkCell> wc, const std::string& deviceName, unsigned int fixateJoint = 100);

	//! @brief Destructor.
	virtual ~KukaIIWAPlanner();

	/**
	 * @brief Solve the inverse kinematics problem for a specific cartesian location.
	 *
	 * The solutions are ordered such that the ones closest to the current state is returned first.
	 *
	 * @param baseTend [in] the target location of the robots end frame relative to its base.
	 * @param state [in] the state with the current configuration.
	 * @return the found joint space solutions. The vector can be empty if no solution is found.
	 */
	std::vector<rw::math::Q> solve(const rw::math::Transform3D<>& baseTend, const rw::kinematics::State& state) const;

	/**
	 * @brief Solve the inverse kinematics problem for a specific cartesian location.
	 *
	 * The solutions are ordered such that the ones closest to the current state is returned first.
	 *
	 * @param baseTframe [in] the target location of the \b frame relative to the robot base.
	 * @param frame [in] the frame to find solution for.
	 * @param state [in] the state with the current configuration.
	 * @return the found joint space solutions. The vector can be empty if no solution is found.
	 */
	std::vector<rw::math::Q> solve(const rw::math::Transform3D<>& baseTframe, const rw::kinematics::Frame* frame, const rw::kinematics::State& state) const;

	/**
	 * @brief Plan a collision free path to a specific cartesian location.
	 * @param baseTend [in] the target location of the robots end frame relative to its base.
	 * @param state [in] the state with the current configuration.
	 * @return the found joint space path. The vector can be empty if no path found.
	 */
	std::vector<rw::math::Q> plan(const rw::math::Transform3D<>& baseTend, const rw::kinematics::State& state) const;

	/**
	 * @brief Plan a linear path in cartesian space.
	 * @param baseTframe [in] the target location of the \b frame relative to the robot base.
	 * @param frame [in] the frame to move linearly.
	 * @param state [in] the state with the current configuration.
	 * @param collisionCheck [in] make sure the path is collision free. This is the default.
	 * @return the found joint space path. The vector can be empty if no path found.
	 */
	std::vector<rw::math::Q> planLinear(const rw::math::Transform3D<>& baseTframe, const rw::kinematics::Frame* frame,  const rw::kinematics::State& state, bool collisionCheck = true) const;

	/**
	 * @brief Plan a collision free path to a specific joint space location.
	 * @param qTo [in] the target configuration.
	 * @param state [in] the state with the current configuration.
	 * @return the found joint space path. The vector can be empty if no path found.
	 */
	std::vector<rw::math::Q> plan(const rw::math::Q& qTo, const rw::kinematics::State& state) const;

private:
	std::vector<rw::math::Q> orderSolutions(const std::vector<rw::math::Q>& in, const rw::math::Q& from) const;

	const rw::common::Ptr<rw::models::WorkCell> _wc;
	const rw::common::Ptr<rw::models::SerialDevice> _device;
	const rw::common::Ptr<rw::models::Device> _fixatedDevice;
	rw::invkin::ClosedFormIKSolverKukaIIWA* const _solver;
	rw::invkin::InvKinSolver* const _jacSolver;
	rw::proximity::CollisionDetector* const _detector;
	rw::pathplanning::PlannerConstraint* _constraint;
	rw::common::Ptr<rw::pathplanning::QToQPlanner> _planner;
	rwlibs::pathoptimization::PathLengthOptimizer* _lengthOpt;
};

#endif /* CAROS_KUKABROKER_DEMO_KUKAIIWAPLANNER_HPP_ */

