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

#include "KukaIIWAPlanner.hpp"

#include <rw/invkin/ClosedFormIKSolverKukaIIWA.hpp>
#include <rw/invkin/JacobianIKSolver.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/math/MetricFactory.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/pathplanning/PlannerUtil.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>
#include <rwlibs/pathoptimization/pathlength/PathLengthOptimizer.hpp>

using rw::common::ownedPtr;
using namespace rw::invkin;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using rwlibs::pathplanners::RRTPlanner;
using rwlibs::proximitystrategies::ProximityStrategyPQP;
using rwlibs::pathoptimization::PathLengthOptimizer;

namespace {
// Hijacking of the JacobianCalculator to allow fixation of a joint.
class FixedJointJacobianCalculator: public JacobianCalculator {
public:
	FixedJointJacobianCalculator(const JacobianCalculator::Ptr calculator, unsigned int fixedJoint): _calc(calculator), _fixedJoint(fixedJoint) {}
	virtual ~FixedJointJacobianCalculator() {}
	Jacobian get(const State& state) const {
		Jacobian res = _calc->get(state);
		if (res.size2() > _fixedJoint) {
		    Eigen::MatrixXd red(6,6);
		    red << res.e().leftCols(_fixedJoint), res.e().rightCols(6-_fixedJoint);
		    res = Jacobian(red);
	    }
		return res;
	}

private:
    const JacobianCalculator::Ptr _calc;
	const unsigned int _fixedJoint;
};

// Wrapper for a device where one joint is fixed (for use with jacobian based solvers).
class KukaFixedJointDevice: public Device {
public:
	KukaFixedJointDevice(const Device::Ptr device, unsigned int fixedJoint): Device(device->getName()), _device(device), _fixedJoint(fixedJoint) {}
	virtual ~KukaFixedJointDevice(){}
	// Utility functions for reducing/extending the configuration vector
	Q extQ(const Q& q, State& state) const {
	    Q qext = _device->getQ(state);
	    for (unsigned int i = 0; i < _fixedJoint; i++) {
	        qext[i] = q[i];
	    }
	    for (unsigned int i = _fixedJoint+1; i < 7; i++) {
	        qext[i] = q[i-1];
	    }
	    return qext;
	}
	Q redQ(const Q& full) const {
	    Q q(6);
	    for (unsigned int i = 0; i < _fixedJoint; i++) {
	        q[i] = full[i];
	    }
	    for (unsigned int i = _fixedJoint+1; i < 7; i++) {
	        q[i-1] = full[i];
	    }
	    return q;
	}
	// Modify the important functions used by the JacobianIKSolver (to simulate a device with less DOFs)
	void setQ(const Q& q, State& state) const {
	    _device->setQ(extQ(q,state),state);
	}
	Q getQ(const State& state) const {
	    return redQ(_device->getQ(state));
	}
	QBox getBounds() const { return QBox(redQ(_device->getBounds().first),redQ(_device->getBounds().second)); }
	JacobianCalculator::Ptr baseJCend(const State& state) const {
	    return ownedPtr(new FixedJointJacobianCalculator(_device->baseJCend(state), _fixedJoint));
	}
    // Forward following functions directly to wrapped device
	void setBounds(const QBox& bounds) { _device->setBounds(bounds); }
	Q getVelocityLimits() const { return _device->getVelocityLimits(); }
	void setVelocityLimits(const Q& vellimits) { _device->setVelocityLimits(vellimits); }
	Q getAccelerationLimits() const { return _device->getAccelerationLimits(); }
	void setAccelerationLimits(const Q& acclimits) { _device->setAccelerationLimits(acclimits); }
	size_t getDOF() const { return _device->getDOF(); }
	Frame* getBase() { return _device->getBase(); }
	const Frame* getBase() const { return _device->getBase(); }
	Frame* getEnd() { return _device->getEnd(); }
	const Frame* getEnd() const { return _device->getEnd(); }
	// Functions for jacobian that should not be used
	Jacobian baseJend(const State& state) const {
	    RW_THROW("Not implemented!");
	    return _device->baseJend(state);
	}
	Jacobian baseJframe(const Frame* frame, const State& state) const {
	    RW_THROW("Not implemented!");
	    return _device->baseJframe(frame, state);
	}
	Jacobian baseJframes(const std::vector<Frame*>& frames, const State& state) const {
	    RW_THROW("Not implemented!");
	    return _device->baseJCframes(frames, state)->get(state);
	}
	JacobianCalculator::Ptr baseJCframe(const Frame* frame, const State& state) const {
	    RW_THROW("Not implemented!");
	    return _device->baseJCframe(frame, state);
	}
	JacobianCalculator::Ptr baseJCframes(const std::vector<Frame*>& frames, const State& state) const {
	    RW_THROW("Not implemented!");
	    return _device->baseJCframes(frames, state);
	}

private:
	const Device::Ptr _device;
    const unsigned int _fixedJoint;
};
}

KukaIIWAPlanner::KukaIIWAPlanner(rw::common::Ptr<WorkCell> wc, const std::string& deviceName, unsigned int fixateJoint):
	_wc(wc),
	_device(_wc->findDevice<SerialDevice>(deviceName)),
	_fixatedDevice(fixateJoint > 6 ? NULL : ownedPtr(new KukaFixedJointDevice(_device,fixateJoint))),
	_solver(new ClosedFormIKSolverKukaIIWA(_device,_wc->getDefaultState())),
	_jacSolver(new JacobianIKSolver(fixateJoint > 6 ? _device : _fixatedDevice,_wc->getDefaultState())),
	_detector(new CollisionDetector(_wc, ProximityStrategyPQP::make()))
{
	static const double EDGE_RESOLUTION = 0.01; // normalizing infinity metric
	static const double EXTEND = 0.1; // weighted euclidean metric

	const State state = _wc->getDefaultState();

	{
		const QConstraint::Ptr qConstraint = QConstraint::make(_detector,_device,state);
		const QMetric::Ptr metric = PlannerUtil::normalizingInfinityMetric(_device->getBounds());
		const QEdgeConstraint::Ptr edge = QEdgeConstraint::make(qConstraint, metric, EDGE_RESOLUTION);
		_constraint = new PlannerConstraint(qConstraint,edge);
	}
	const QSampler::Ptr sampler = QSampler::makeUniform(_device);
	const QMetric::Ptr metric = MetricFactory::makeWeightedEuclidean(Q(7,1,1,1,1,1,1,1));

	_planner = RRTPlanner::makeQToQPlanner(*_constraint, sampler, metric, EXTEND, RRTPlanner::RRTConnect);
	_lengthOpt = new PathLengthOptimizer(*_constraint, metric);

	_solver->setCheckJointLimits(true);
	_jacSolver->setCheckJointLimits(true);
}

KukaIIWAPlanner::~KukaIIWAPlanner() {
	delete _solver;
	delete _detector;
	delete _constraint;
	delete _lengthOpt;
}

std::vector<Q> KukaIIWAPlanner::solve(const Transform3D<>& baseTend, const State& state) const {
	const Q from = _device->getQ(state);
	QPath path;
	std::vector<Q> solutions;
	for (std::size_t i = 0; i < 500 && solutions.size() < 100; i++) {
		const std::vector<Q> sols = _solver->solve(baseTend,state);
		solutions.insert(solutions.end(),sols.begin(),sols.end());
	}
	return orderSolutions(solutions,from);
}

std::vector<Q> KukaIIWAPlanner::solve(const Transform3D<>& baseTframe, const Frame* const frame, const State& state) const {
	const Transform3D<> frameTend = Kinematics::frameTframe(frame,_device->getEnd(),state);
	return solve(baseTframe*frameTend,state);
}

std::vector<Q> KukaIIWAPlanner::plan(const Transform3D<>& baseTend, const State& state) const {
	const std::vector<Q> solutions = solve(baseTend,state);
	std::vector<Q> path;
	for (std::size_t i = 0; i < solutions.size() && path.size() == 0; i++) {
		for (unsigned int k = 0; k < 10 && path.size() == 0; k++) {
			path = plan(solutions[i],state);
		}
	}
	return path;
}

std::vector<Q> KukaIIWAPlanner::planLinear(const Transform3D<>& baseTframe, const Frame* const frame, const State& state, bool collisionCheck) const {
	Q from = _device->getQ(state);
	const Transform3D<> Tfrom = _device->baseTframe(frame,state);
	const Transform3D<> frameTend = Kinematics::frameTframe(frame,_device->getEnd(),state);
	const LinearInterpolator<Transform3D<> > interpolator(Tfrom,baseTframe,1);
	State tmpState = state;
	QPath path;
	path.push_back(from);
	for (std::size_t timeI = 0; timeI <= 100; timeI++) {
		Transform3D<> T;
		if (timeI > 0 && timeI < 100)
			T = interpolator.x(0.01*timeI);
		else if (timeI == 0)
			T = interpolator.getStart();
		else if (timeI >= 100)
			T = interpolator.getEnd();
		const Transform3D<> baseTend = T*frameTend;
		// Find reduced solutions
		std::vector<Q> solRed = _jacSolver->solve(baseTend,tmpState);
		// Expand solutions to full configuration vector
		std::vector<Q> solutions;
		if (!_fixatedDevice.isNull()) {
    		const rw::common::Ptr<const KukaFixedJointDevice> fixatedDevice = _fixatedDevice.cast<const KukaFixedJointDevice>();
		    for (std::size_t i = 0; i < solRed.size(); i++) {
		        solutions.push_back(fixatedDevice->extQ(solRed[i],tmpState));
		    }
		} else {
		    solutions = solRed;
		}
		solutions = orderSolutions(solutions,from);
		if (solutions.size() == 0)
			return std::vector<Q>();
		if (collisionCheck) {
			_constraint->getQConstraint().update(state);
			if (_constraint->inCollision(solutions.front()))
				return std::vector<Q>();
			else if (_constraint->inCollision(from,solutions.front()))
				return std::vector<Q>();
		}
		path.push_back(solutions.front());
		from = solutions.front();
		_device->setQ(from,tmpState);
	}
	return path;
}

std::vector<Q> KukaIIWAPlanner::plan(const Q& qTo, const State& state) const {
	const Q from = _device->getQ(state);
	QPath path;
	_constraint->getQConstraint().update(state);
	if (_constraint->inCollision(qTo)) {
		return path;
	} else if (!_constraint->inCollision(from,qTo)) {
		path.push_back(from);
		path.push_back(qTo);
	} else {
		_planner->query(from, qTo, path, 1);
	}
	if (path.size() > 2)
		path = _lengthOpt->pathPruning(path);
	return path;
}

std::vector<Q> KukaIIWAPlanner::orderSolutions(const std::vector<Q>& in, const Q& from) const {
	std::vector<Q> ordered;
	if (in.size() == 0)
		return ordered;
	Q velInv(7);
	for (std::size_t i = 0; i < 7; i++) {
		velInv[i] = 1./_device->getVelocityLimits()[i];
	}
	const WeightedInfinityMetric<Q> metric(velInv);

	std::list<Q> remaining;
	for (std::size_t i = 0; i < in.size(); i++)
		remaining.push_back(in[i]);

	while(remaining.size() > 0) {
		std::list<Q>::iterator min = remaining.begin();
		double dist = metric.distance(from,*min);
		std::list<Q>::iterator it = remaining.begin();
		for (it++; it != remaining.end(); it++) {
			const double d = metric.distance(from,*it);
			if (d < dist) {
				min = it;
				dist = d;
			}
		}
		ordered.push_back(*min);
		remaining.erase(min);
	}

	return ordered;
}

