/*
 * ActionIKSolver.hpp
 *
 *  Created on: Jan 19, 2015
 *      Author: jpb
 */

#ifndef ACTIONIKSOLVER_HPP_
#define ACTIONIKSOLVER_HPP_

#include <string>
#include <rw/math.hpp>
#include <rw/trajectory.hpp>
#include <rw/invkin/JacobianIKSolver.hpp>
//#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rw/trajectory/Path.hpp>
#include <rw/common/Ptr.hpp>
#include <rw/models/WorkCell.hpp>


namespace dsl {

class ActionIKSolver {
public:
	typedef rw::common::Ptr<ActionIKSolver> Ptr;
	ActionIKSolver(std::string wcStig, std::string deviceName, std::string toolFrame, std::string refFrame);

	void setInitialQ(rw::math::Q q);
	void setOffsetTransform(rw::math::Transform3D<> offset);
	void setOffsetTransformIdentity();

	rw::trajectory::TimedQPath solve(std::string trajectoryFileName);
	rw::trajectory::TimedQPath solve(rw::trajectory::Path<rw::trajectory::Timed<rw::math::Transform3D<> > > timedT3DPath);
	void saveToFile(std::string stig);


private:
	rw::models::WorkCell::Ptr _wc;
	rw::models::Device::Ptr _device;
	rw::kinematics::State _state;
	rw::kinematics::Frame * _tool;
	rw::kinematics::Frame * _ref;
	rw::invkin::JacobianIKSolver* _invKin;

	rw::math::Transform3D<double> _offsetTransform;
};

}

#endif /* ACTIONIKSOLVER_HPP_ */
