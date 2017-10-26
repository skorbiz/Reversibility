/*
 * WorkcellModel.cpp
 *
 *  Created on: Jul 30, 2014
 *      Author: josl
 */

#include "WorkcellModel.hpp"

namespace dsl {
namespace common {
using namespace rw::math;

WorkcellModel::WorkcellModel()
{

//  nameProjectRootPath = ros::package::getPath("initialDemo");
//	nameWokcellFilePath	= "../scenes/KVM_pih_scene/pih.wc.xml";//"src/simulation/KVM_pih_scene/pih.wc.xml";
//	std::cout << nameProjectRootPath << "+++" << nameWokcellFilePath << std::endl;
    std::string path = dsl::common::getPath2trunk() + "/scenes/CARMENscene_3.0/Scene_DSL_apr15.wc.xml";
	nameDevice			= "UR5";
	nameFrameBase		= "Base";
	nameFrameToolmount	= "TCP";
	nameFrameTCP_KVM	= "TCP_KVM";
	nameFrameTCP_VOLA	= "TCP_Screwdriver";

//    workcell = rw::loaders::WorkCellLoader::Factory::load(nameProjectRootPath + nameWokcellFilePath);
    workcell = rw::loaders::WorkCellLoader::Factory::load(path);
    device 			= workcell->findDevice(nameDevice);
	state 			= workcell->getDefaultState();
    frameBase 		= workcell->findFrame(nameDevice + "." + nameFrameBase);
	frameToolmount 	= workcell->findFrame(nameDevice + "." + nameFrameToolmount);
	frameTCP_KVM	= workcell->findFrame(nameFrameTCP_KVM);
	frameTCP_VOLA	= workcell->findFrame(nameFrameTCP_VOLA);

	collisionDetector = rw::proximity::CollisionDetector::Ptr(new rw::proximity::CollisionDetector(workcell));

	if (workcell == nullptr) {std::cerr << "Workcell: " << nameWokcellFilePath << " not found!" << std::endl;		exit(-1);}
	if (device == nullptr) {std::cerr << "Device: " << nameDevice << " not found!" << std::endl;					exit(-1);}
	if (frameBase == nullptr) {std::cerr << "Frame: " << nameFrameBase << " not found!" << std::endl;				exit(-1);}
	if (frameToolmount == nullptr)	{std::cerr << "Frame: " << nameFrameToolmount << " not found!" << std::endl;	exit(-1);}
	if (frameTCP_KVM == nullptr) {std::cerr << "Frame: " << nameFrameTCP_KVM << " not found!" << std::endl;			exit(-1);}
	if (frameTCP_VOLA == nullptr) {std::cerr << "Frame: " << nameFrameTCP_VOLA << " not found!" << std::endl;		exit(-1);}
}

WorkcellModel::~WorkcellModel()
{
}


void WorkcellModel::setRobotJointConfiguration(rw::math::Q q)
{
//	rw::math::Q qDefault = device->getQ(state);
//	qDefault[3] = -3.158;ip
	device->setQ(q,state);
}

rw::math::Q WorkcellModel::getRobotJointConfiguration()
{
	return device->getQ(state);
}


std::vector<rw::math::Q> WorkcellModel::getNewConfigurationsForTransform(rw::math::Transform3D<> baseT2end)
{
	rw::invkin::JacobianIKSolver invKin(device, state);
	invKin.setSolverType(invKin.SVD);
	std::vector<rw::math::Q> qTargetSolutions = invKin.solve(baseT2end, state);//baseTAttempted,state);
//	std::cout << "qTargetSolutions found: " << qTargetSolutions.size() << std::endl;
//	for(unsigned int i = 0; i < qTargetSolutions.size(); i++)
//		std::cout << qTargetSolutions[i] << std::endl;
	return qTargetSolutions;
}

rw::kinematics::Frame::Ptr WorkcellModel::getFrameRW(frame aFrame) const
{
	rw::kinematics::Frame::Ptr rwFrame;
	switch (aFrame){
		case frame::base: 		rwFrame = frameBase;		break;
		case frame::toolmount: 	rwFrame = frameToolmount;	break;
		case frame::tcp_kvm:	rwFrame = frameTCP_KVM;	    break;
		case frame::tcp_vola:	rwFrame = frameTCP_VOLA;	break;
		default: std::cerr << "Invalid frame chosen in workcellmodel getFrameRW" << std::endl;	exit(-1);
	}
	return rwFrame;
}

rw::kinematics::Frame::Ptr WorkcellModel::getFrameRW(std::string aFrame) const
{
	return workcell->findFrame(aFrame);
}


rw::math::Transform3D<> WorkcellModel::getTransformParentTChild(frame parent, frame child) const
{
	rw::kinematics::Frame::Ptr parentRWframe = getFrameRW(parent);
	rw::kinematics::Frame::Ptr childRWframe = getFrameRW(child);
	return getTransformParentTChild(parentRWframe, childRWframe);
}

rw::math::Transform3D<> WorkcellModel::getTransformParentTChild(const rw::kinematics::Frame::Ptr & parent, const rw::kinematics::Frame::Ptr & child) const
{
	assert(!parent.isNull());
	assert(!child.isNull());

	rw::math::Transform3D<> parentTransformChild;
	rw::kinematics::Frame::Ptr tempFrame = child;

	while(true)
	{
		parentTransformChild = tempFrame->getTransform(state) * parentTransformChild;
		tempFrame = tempFrame->getParent();

		if(tempFrame == parent)
			break;

		if(tempFrame == workcell->getWorldFrame()){
			std::cerr << "Parent frame \"" << parent->getName() << "\" was not a parent to the child frame \"" << child->getName() << "\"" << std::endl; exit(-1);}
	}

	return parentTransformChild;
}




bool WorkcellModel::isCollisionFree()
{
	bool inCollision = collisionDetector->inCollision(state);
	return !inCollision;
	return true;
}

bool WorkcellModel::pathJointCollisionFree(rw::math::Q qStart, rw::math::Q qEnd)
{
	const double EPSILON = 0.01;
	rw::math::Q dq = qEnd-qStart;
	int n = dq.norm2()/EPSILON;

	int levels = ceil(log2(n));

	for(int i = 1; i <= levels; i++)
	{
		double steps = pow(2.0, i-1);
		rw::math::Q step = dq/steps;

		for( int j = 1; j <= steps; j++)
		{
			rw::math::Q qi = qStart + (j-0.5)*step;
			setRobotJointConfiguration(qi);
			if(!isCollisionFree())
				return false;
		}
	}
	return true;
}




} /* namespaace common */
} /* namespace dsl */
