/*
 * WorkcellModel.hpp
 *
 *  Created on: Jul 30, 2014
 *      Author: josl
 */

#ifndef WORKCELLMODEL_HPP_
#define WORKCELLMODEL_HPP_

#include <iostream>
#include <ros/package.h>
#include <rw/rw.hpp>
#include <rw/math/RPY.hpp>
#include <rw/math.hpp>
#include <rw/invkin/JacobianIKSolver.hpp>
#include <rw/invkin/ClosedFormIKSolverUR.hpp>
#include <rw/proximity.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
#include <../src/common/common.hpp>

namespace dsl {
namespace common {
using namespace rw::math;

class WorkcellModel
{

public:

	enum class frame {base, toolmount, tcp_vola, tcp_kvm};

	 WorkcellModel();
	~WorkcellModel();

	void setRobotJointConfiguration(rw::math::Q q);
	rw::math::Q getRobotJointConfiguration();

	std::vector<rw::math::Q> getNewConfigurationsForTransform(rw::math::Transform3D<> baseT2end);
	std::vector<rw::math::Q> getNewConfigurationsForProjectedFrame(frame aframe, rw::math::Vector3D<> moveDirection);

	rw::kinematics::Frame::Ptr getFrameRW(frame aFrame) const;
	rw::kinematics::Frame::Ptr getFrameRW(std::string aFrame) const;

    rw::math::Transform3D<> getTransformParentTChild(frame parent, frame child) const;
    rw::math::Transform3D<> getTransformParentTChild(const rw::kinematics::Frame::Ptr & parent, const rw::kinematics::Frame::Ptr & child) const;

    bool isCollisionFree();
    bool pathJointCollisionFree(rw::math::Q qStart, rw::math::Q qEnd);

private:
    std::string nameProjectRootPath;
	std::string nameWokcellFilePath;
	std::string nameDevice;
	std::string nameFrameBase;
	std::string nameFrameToolmount;
	std::string nameFrameTCP_KVM;
	std::string nameFrameTCP_VOLA;

    rw::models::WorkCell::Ptr workcell;
    rw::models::Device::Ptr device;
	rw::kinematics::State state;
    rw::kinematics::Frame::Ptr frameBase;
	rw::kinematics::Frame::Ptr frameToolmount;
	rw::kinematics::Frame::Ptr frameTCP_KVM;
	rw::kinematics::Frame::Ptr frameTCP_VOLA;

	rw::proximity::CollisionDetector::Ptr collisionDetector;
};

} /* namespace common */
} /* namespace dsl */

#endif /* WORKCELLMODEL_HPP_ */



