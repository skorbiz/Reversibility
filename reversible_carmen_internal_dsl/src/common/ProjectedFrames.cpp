/*
 * ProjectedFrames.cpp
 *
 *  Created on: Sep 15, 2015
 *      Author: josl
 */

#include "ProjectedFrames.hpp"

namespace dsl {
namespace common {

ProjectedFrames::ProjectedFrames(dsl::common::WorkcellModel wc) :
		_wc(wc)
{
}

ProjectedFrames::~ProjectedFrames()
{
}


rw::math::Transform3D<> ProjectedFrames::getTransformBaseTNew(WorkcellModel::frame aframe, rw::math::Vector3D<> moveDirection)
{

	rw::kinematics::Frame::Ptr frameSelected;
	rw::kinematics::Frame::Ptr frameBase;
	frameSelected = _wc.getFrameRW(aframe);
	frameBase = _wc.getFrameRW(WorkcellModel::frame::base);

	rw::math::Transform3D<> baseTselected = _wc.getTransformParentTChild(frameBase, frameSelected);
	rw::math::Transform3D<> baseTprojected = getTransformBaseTProjected(baseTselected);

//	Transform3D<> baseTtoolmount = getTransformParentTChild(frameBase, frameToolmount);
//	Transform3D<> toolMountTprojected = baseTprojected * inverse(baseTtoolmount);

	rw::math::Transform3D<> projectedTprojectedTarget(moveDirection);
	rw::math::Transform3D<> projectedTselected = inverse(baseTprojected) * baseTselected;

	rw::math::Transform3D<> baseTprojectedTarget = baseTprojected * projectedTprojectedTarget;

	rw::math::Transform3D<> baseTselectedTarget(baseTprojectedTarget.P() + projectedTselected.P(), baseTselected.R());


	std::cout << "projectedTprojectedTarget.P:  " << projectedTprojectedTarget.P() << std::endl;
	std::cout << "projectedTprojectedTarget.R:  " << projectedTprojectedTarget.R() << std::endl;
	std::cout << std::endl;

	std::cout << "baseTselected.P:		  " << baseTselected.P() << std::endl;
	std::cout << "baseTprojected.P:		  " << baseTprojected.P() << std::endl;
	std::cout << "baseTprojectedTarget.P: " << baseTprojectedTarget.P() << std::endl;
	std::cout << std::endl;

	std::cout << "baseTselected.R:		  " << baseTselected.R() << std::endl;
	std::cout << "baseTprojected.R:		  " << baseTprojected.R() << std::endl;
	std::cout << "baseTprojectedTarget.R: " << baseTprojectedTarget.R() << std::endl;
	std::cout << std::endl;

	std::cout << "projectedTselected.P:  " << projectedTselected.P() << std::endl;
	std::cout << "projectedTselected.R:  " << projectedTselected.R() << std::endl;
	std::cout << std::endl;

	std::cout << "baseTselectedTarget.P:  " << baseTselectedTarget.P() << std::endl;
	std::cout << "baseTselectedTarget.R:  " << baseTselectedTarget.R() << std::endl;
	std::cout << std::endl;

//	std::cout << "q current: " << device->getQ(state) << std::endl;
//	std::cout << "frame base: " << device->getBase()->getName() << std::endl;
//	std::cout << "frame end: " << device->getEnd()->getName() << std::endl;
//	std::cout << "frame selected: " << frameSelected->getName() << std::endl;
//	std::cout << std::endl;

	return baseTselectedTarget;

//	std::vector<rw::math::Q> qTargetSolutions = WorkcellModel::getNewConfigurationsForTransform(baseTselectedTarget);
//
//	std::cout << "qTargetSolutions found: " << qTargetSolutions.size() << std::endl;
//	for(unsigned int i = 0; i < qTargetSolutions.size(); i++)
//		std::cout << qTargetSolutions[i] << std::endl;
//
//	return qTargetSolutions;
}

rw::math::Transform3D<> ProjectedFrames::getTransformBaseTProjected(const rw::math::Transform3D<> & baseTselected)
{
	//Vectors at base frame
	rw::math::Vector3D<> p0_B(0,0,0);
	rw::math::Vector3D<> xUnit_B(1,0,0);
	rw::math::Vector3D<> yUnit_B(0,1,0);
	rw::math::Vector3D<> zUnit_B(0,0,1);

	//Vectors at Selected frame
	rw::math::Vector3D<> p0_S = baseTselected * p0_B;
	rw::math::Vector3D<> zUnit_S = baseTselected * zUnit_B - p0_S;

	//Vectors at Projected frame
	rw::math::Vector3D<> p0_P = p0_S;
	rw::math::Vector3D<> normal_P(0,0,1);
	rw::math::Vector3D<> zProjected = zUnit_S - dot(zUnit_S, normal_P)/dot(normal_P,normal_P)*normal_P;
	rw::math::Vector3D<> zUnit_P = zProjected/zProjected.norm2();
	rw::math::Vector3D<> yUnit_P(0,0,1);
	rw::math::Vector3D<> xUnit_P = cross(yUnit_P, zUnit_P);

	//Calculating rotation to projected frame using svd
	//	Described in http://nghiaho.com/?page_id=671
	//	Least-Squares Fitting of Two 3-D Point Sets” by K.S Arun et. alii (1987 )
	rw::math::Vector3D<> centroid_B = (xUnit_B + yUnit_B + zUnit_B)/3;
	rw::math::Vector3D<> centroid_P = (xUnit_P + yUnit_P + zUnit_P)/3 + p0_P;

	Eigen::Matrix<double,3,3> H;
	H  = (xUnit_B.e() - centroid_B.e()) * (xUnit_P.e()+p0_P.e() - centroid_P.e()).transpose();
	H += (yUnit_B.e() - centroid_B.e()) * (yUnit_P.e()+p0_P.e() - centroid_P.e()).transpose();
	H += (zUnit_B.e() - centroid_B.e()) * (zUnit_P.e()+p0_P.e() - centroid_P.e()).transpose();

	Eigen::MatrixXd U;
	Eigen::VectorXd sigma;
	Eigen::MatrixXd V;
	rw::math::LinearAlgebra::svd(H,U,sigma,V);

	Eigen::Matrix<double,3,3> rotation_base2projected =  V * U.transpose();

	if(rotation_base2projected.determinant() < 0)
	{
		std::cout << "rotation determinant < 0" << std::endl << " special case with mirrored solution prevented" << std::endl;
		for(int row = 0; row < 3; row++)
			V(row,2) *=-1.0;
		rotation_base2projected = V * U.transpose();
	}

	rw::math::Transform3D<> baseTprojected(p0_P, rw::math::Rotation3D<>(rotation_base2projected));

	double r11 = rotation_base2projected(0,0);
	double r21 = rotation_base2projected(1,0);
	double r31 = rotation_base2projected(2,0);
	double r32 = rotation_base2projected(2,1);
	double r33 = rotation_base2projected(2,2);

	double angleY = asin(-r31);
	double angleX = atan2(cos(angleY)*r32,cos(angleY)*r33);
	double angleZ = atan2(cos(angleY)*r21,cos(angleY)*r11);

	if(false)
	{
		std::cout << "Rotation calulation output:::::::::::::::::" << std::endl;

		std::cout << "Vectors at base frame:" << std::endl;
		std::cout << "p0_B: \t" << p0_B << std::endl;
		std::cout << "xUnit_B: \t" << xUnit_B << std::endl;
		std::cout << "yUnit_B: \t" << yUnit_B << std::endl;
		std::cout << "zUnit_B: \t" << zUnit_B << std::endl;
		std::cout << std::endl;

		std::cout << "Vectors at base frame:" << std::endl;
		std::cout << "p0_S: \t" << p0_S << std::endl;
		std::cout << "zUnit_S: \t" << zUnit_S << std::endl;
		std::cout << std::endl;

		std::cout << "Vectors at projected frame:" << std::endl;
		std::cout << "p0_P: \t" << p0_P << std::endl;
		std::cout << "normal_P: \t" << normal_P << std::endl;
		std::cout << "zProjected: \t" << zProjected << std::endl;
		std::cout << "zUnit_P: \t" << zUnit_P << std::endl;
		std::cout << "yUnit_P: \t" << yUnit_P << std::endl;
		std::cout << "xUnit_P: \t" << xUnit_P << std::endl;
		std::cout << std::endl;

		std::cout << "Norm check of unit vectors:" << std::endl;
		std::cout << "zUnit_B: \t" << zUnit_B.norm2() << std::endl;
		std::cout << "zUnit_S: \t" << zUnit_S.norm2() << std::endl;
		std::cout << "zUnit_P: \t" << zUnit_P.norm2() << std::endl;
		std::cout << "zProjected ( != 1 ): \t" << zProjected.norm2() << std::endl;
		std::cout << std::endl;

		std::cout << "Rotation calculations:" << std::endl;
		std::cout << "centroid_B: \t" << centroid_B << std::endl;
		std::cout << "centroid_P: \t" << centroid_P << std::endl;
		std::cout << "H: \t" << std::endl << H << std::endl;
		std::cout << "U: \t" << std::endl << U << std::endl;
		std::cout << "sigma: \t" << sigma << std::endl;
		std::cout << "V: \t" << std::endl << V << std::endl;
		std::cout << std::endl;

		std::cout << "Final Transfor:" << std::endl;
		std::cout << "baseTprojected: \t" << std::endl << baseTprojected << std::endl;
		std::cout << std::endl;

		std::cout << "Final rotation angles:" << std::endl;
		std::cout << "angleZ: \t" << angleZ * rw::math::Rad2Deg << std::endl;
		std::cout << "angleY: \t" << angleY * rw::math::Rad2Deg << std::endl;
		std::cout << "angleX: \t" << angleX * rw::math::Rad2Deg << std::endl;
		std::cout << std::endl;

//		std::cout << "Robot jointconfiguration:" << std::endl;
//		std::cout << "{";
//		for(int i=0; i < 7; i++){ std::cout << device->getQ(state)[i]; if(i!=6) std::cout << ",";}
//		std::cout << "}" << std::endl;
//		std::cout << std::endl;
	}

	return baseTprojected;
}









} /* namespace common */
} /* namespace dsl */
