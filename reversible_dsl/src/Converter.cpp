/*
 * Converter.cpp
 *
 *  Created on: Mar 10, 2017
 *      Author: josl
 */

#include "Converter.h"
#include <ros/ros.h>

Converter::Converter(rw::models::WorkCell::Ptr wc,
		  rw::models::Device::Ptr device,
		  std::shared_ptr<rw::kinematics::Frame> toolFrame) :
		wc(wc),
		device(device),
		toolFrame(toolFrame)
{
}

Converter::~Converter()
{
}



rw::math::Transform3D<> Converter::frameTframe(std::shared_ptr<rw::kinematics::Frame> from, std::shared_ptr<rw::kinematics::Frame> to, const rw::kinematics::State& state) const
{
	return rw::kinematics::Kinematics::frameTframe(from.get(), to.get(), state);
}

rw::math::Transform3D<> Converter::toBaseTtool(const rw::math::Transform3D<> toolTobject, const std::shared_ptr<rw::kinematics::Frame> objTarget, const rw::kinematics::State state) const
{
	rw::math::Transform3D<>	baseTobject = rw::kinematics::Kinematics::frameTframe(device->getBase(), objTarget.get(), state);
	rw::math::Transform3D<> baseTtool = baseTobject * inverse(toolTobject);
	return baseTtool;
}

rw::math::Transform3D<> Converter::toBaseTtool(rw::math::Q q, const rw::kinematics::State state_input) const
{
	rw::kinematics::State state = state_input;
	device->setQ(q,state);
	return device->baseTframe(toolFrame.get(),state);
}

rw::math::Transform3D<> Converter::toBaseTtool( const rw::kinematics::State state) const
{
	return device->baseTframe(toolFrame.get(),state);
}

rw::math::Q Converter::toQ( const rw::kinematics::State state) const
{
	return device->getQ(state);
}

rw::math::Transform3D<> Converter::rotated_deg(double rz, double ry, double rx, rw::math::Transform3D<> input)
{
	double deg2rad = M_PI/180.0;
	rw::math::RPY<> rpy(rz*deg2rad, ry*deg2rad, rx*deg2rad);
	rw::math::Transform3D<> inputTrotated(rpy.toRotation3D());
	return input*inputTrotated;
}


void Converter::toScreen(std::string name, rw::math::Transform3D<> transform) const
{
	const rw::math::Vector3D<> pos = transform.P();
	const rw::math::RPY<> rpy(transform.R());
	ROS_INFO_STREAM( name <<": {x,y,z,R,P,Y}     = {" << pos[0] << ", " << pos[1] << ", " << pos[2] << ", " << rpy[0] << ", " << rpy[1] << ", " << rpy[2] << "}");
	//ROS_INFO_STREAM( "- - - RPY("<<rpy[0]*180.0/M_PI << ", " << rpy[1]*180.0/M_PI << ", " << rpy[2]*180.0/M_PI << ")");
}



