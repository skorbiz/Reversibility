/*
 * VisionAct.cpp
 *
 *  Created on: Sep 7, 2015
 *      Author: josl
 */

#include "VisionAct.hpp"
namespace dsl {

bool VisionAct::wayToSort(int i, int j) { return i > j; }


VisionAct::VisionAct(std::shared_ptr<RobotGeneralInterfaceProxy> RGIP) :
		_RGIP(RGIP),
		_qInitial(6, 0.743, -1.646, 1.909, -1.842, -1.561, -3.968),
//		speed(15.00),	acceleration(2.5)		// Very fast (100 test)
//		speed( 2.00),	acceleration(0.5)		// fast		 (for tested programs)
		_speed( 0.10),	_acceleration(0.1)
//		speed( 0.05),	acceleration(0.1)		//Slow
//		speed( 0.01),	acceleration(0.1)		//Painfully slow
{
	calculate();
	execute();
}

VisionAct::~VisionAct()
{
}

void VisionAct::execute()
{
	assert(_qInitial.size() != 0);
	assert(_qAbove.size() != 0);
	assert(_qAt.size() != 0);

	_RGIP->moveJ(_qInitial, _speed, _acceleration);
	_RGIP->moveJ(_qAbove, _speed, _acceleration);
	_RGIP->moveJ(_qAt, _speed, _acceleration);
}

void VisionAct::calculate()
{
	std::cout << _wc.getRobotJointConfiguration() << std::endl;
	std::cout << _wc.getRobotJointConfiguration() * rw::math::Rad2Deg<< std::endl;

	rw::math::Transform3D<> cameraT2object = fetchCameraT2Object();
	rw::math::Transform3D<> worldT2camera = fetchWorldT2Camera();
	rw::math::Transform3D<> worldT2base = fetchWorldT2Base();
	rw::math::Transform3D<>	tcpT2va = fetchUrT2Tcp();
	rw::math::Transform3D<> baseT2object;
	rw::math::Transform3D<> baseT2objectCorrected;
	rw::math::Transform3D<> baseT2Above;
	rw::math::Transform3D<> baseT2AboveCorrected;

	baseT2object = rw::math::inverse(worldT2base) * worldT2camera * cameraT2object;
	baseT2objectCorrected = baseT2object * tcpT2va;
	baseT2Above = baseT2object * fetchObjectT2Above();
	baseT2AboveCorrected = baseT2Above * tcpT2va;

	std::cout << baseT2objectCorrected << std::endl;
	std::cout << baseT2AboveCorrected << std::endl;
	std::cout << fetchObjectT2Above() << std::endl;

	_qAbove = inverseKinimatic(_qInitial, baseT2AboveCorrected);
	_qAt = inverseKinimatic(_qAbove,baseT2objectCorrected);
}

rw::math::Q VisionAct::inverseKinimatic(rw::math::Q init, rw::math::Transform3D<> target)
{
	_wc.setRobotJointConfiguration(init);
	std::vector<rw::math::Q> qVec = _wc.getNewConfigurationsForTransform(target);
	std::sort(qVec.begin(), qVec.end(), [this](rw::math::Q l, rw::math::Q r) {return myComparison(l, r); });

	int pathSelected = -1;
	for(unsigned int i = 0; i < qVec.size(); i++)
		if(_wc.pathJointCollisionFree(init, qVec[i])){
			pathSelected = i;
			break;
		}

	if(pathSelected < 0){
		std::cerr << "Vision Action failed to find collision free path" <<  std::endl;
		exit(-1);
	}

	return qVec[pathSelected];
}

bool VisionAct::myComparison(rw::math::Q i, rw::math::Q j)
{
	double iDifference = (i - _qInitial).size();
	double jDifference = (j - _qInitial).size();
	return iDifference < jDifference;
}

rw::math::Transform3D<> VisionAct::fetchWorldT2Base()
{
	rw::kinematics::Frame::Ptr wld = _wc.getFrameRW("WORLD");
	rw::kinematics::Frame::Ptr bsa = _wc.getFrameRW("UR5.Base");
	return _wc.getTransformParentTChild(wld,bsa);
}

rw::math::Transform3D<> VisionAct::fetchUrT2Tcp()
{
	rw::kinematics::Frame::Ptr tcp = _wc.getFrameRW("TCP_VisionAct");
	rw::kinematics::Frame::Ptr ur  = _wc.getFrameRW("UR5.TCP");
	return _wc.getTransformParentTChild(ur, tcp);
}

rw::math::Transform3D<> VisionAct::fetchObjectT2Above()
{
	rw::math::Transform3D<> objectT2Above;
	objectT2Above(2,3) = 0.1;
	return objectT2Above;
}

rw::math::Transform3D<> VisionAct::fetchCameraT2Object()
{
	rw::kinematics::Frame::Ptr obj = _wc.getFrameRW("object");
	rw::kinematics::Frame::Ptr wld = _wc.getFrameRW("WORLD");
	rw::math::Transform3D<> wldTobj = _wc.getTransformParentTChild(wld, obj);
	rw::math::Transform3D<> wldTcam = fetchWorldT2Camera();
	return rw::math::inverse(wldTcam) * wldTobj;
}

rw::math::Transform3D<> VisionAct::fetchWorldT2Camera()
{
	rw::kinematics::Frame::Ptr cam = _wc.getFrameRW("camera");
	rw::kinematics::Frame::Ptr wld = _wc.getFrameRW("WORLD");
	return _wc.getTransformParentTChild(wld, cam);
}


} /* namespace dsl */
