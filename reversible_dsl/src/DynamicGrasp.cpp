/*
 * DynamicGrasp.cpp
 *
 *  Created on: Jan 24, 2017
 *      Author: josl
 */

#include <memory>
#include "DynamicGrasp.h"

#include "ros/ros.h"
#include <rw/common/TimerUtil.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rwlibs/calibration/xml/XmlCalibrationLoader.hpp>
#include <rw/math/Rotation3D.hpp>

DynamicGrasp::DynamicGrasp(rw::math::Transform3D<> baseTcamera) :
	baseTcamera(baseTcamera)
{
}

DynamicGrasp::~DynamicGrasp()
{
}

bool DynamicGrasp::update(const rw::math::Transform3D<> cameraTobject)
{
	rw::math::Transform3D<> baseTpart = calc_baseTpart(cameraTobject);
	baseTtool = calc_baseTtool(baseTpart);
	baseTtoolRetract = calc_baseTtoolRetract(baseTtool);
	toolTpart = calc_toolTpart(baseTtool, baseTpart);
	return true;
}

rw::math::Transform3D<> DynamicGrasp::calc_baseTpart(const rw::math::Transform3D<> camTpart) const
{
	const rw::math::Transform3D<> baseTpart = baseTcamera*camTpart;
	to_screen("Detected pose from vision server (corrected)", camTpart);
	return baseTpart;
}


rw::math::Transform3D<> DynamicGrasp::calc_baseTtool(const rw::math::Transform3D<> baseTpart) const
{

	const rw::math::Vector3D<> z = rw::math::Vector3D<>::z();
	const rw::math::Vector3D<> y_obj = -baseTpart.R().getCol(1);
	const rw::math::Vector3D<> x_obj = normalize((z-dot(z,y_obj)*y_obj));
	const rw::math::Vector3D<> z_obj = normalize(cross(x_obj,y_obj));
	const rw::math::Rotation3D<> rotation = rw::math::Rotation3D<>(x_obj,y_obj,z_obj);
	rw::math::Transform3D<> baseTtool = rw::math::Transform3D<>(baseTpart.P(),rotation);
	to_screen("Target baseTtool", baseTtool);
	baseTtool.P() += 0.001*baseTtool.R().getCol(0);
	return baseTtool;
}

rw::math::Transform3D<> DynamicGrasp::calc_baseTtoolRetract(const rw::math::Transform3D<> baseTtool) const
{
	rw::math::Transform3D<> retract = baseTtool;
	retract.P() += 0.1*retract.R().getCol(0);
	to_screen("Retracted pose baseTtool", retract);
	return retract;
}

rw::math::Transform3D<> DynamicGrasp::calc_toolTpart(const rw::math::Transform3D<> baseTtool, const rw::math::Transform3D<> baseTpart) const
{
	const rw::math::Transform3D<> toolTpart = inverse(baseTtool)*baseTpart;
	to_screen("toolTpart", toolTpart);
	return toolTpart;
}


void DynamicGrasp::to_screen(std::string t_name, const rw::math::Transform3D<> t) const
{
//	const rw::math::Vector3D<> pos = t.P();
//	const rw::math::RPY<> rpy(t.R());
//	ROS_INFO_STREAM( t_name <<": {x,y,z,R,P,Y} = {" << pos[0] << "," << pos[1] << "," << pos[2] << "," << rpy[0] << "," << rpy[1] << "," << rpy[2] << "}");
}

