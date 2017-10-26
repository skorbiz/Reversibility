/*
 * vision_system.cpp
 *
 *  Created on: Dec 21, 2016
 *      Author: jpb
 */
#include <d7pc_msgs/VisionGraspGetPose.h>
#include <d7pc_msgs/VisionGraspPrepare.h>
#include <VisionSystem.h>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Quaternion.hpp>

VisionSystem::VisionSystem() :
	was_ros_service_succesfull_(false),
	was_get_pose_succesfull_(false)
{
	ros::NodeHandle nh;
	client_get_pose_ = nh.serviceClient<d7pc_msgs::VisionGraspGetPose>("/vision/get_pose");
	client_prepare_ = nh.serviceClient<d7pc_msgs::VisionGraspPrepare>("/vision/prepare");
}

VisionSystem::~VisionSystem()
{
}

bool VisionSystem::update_pose(ObjectType obj)
{
	d7pc_msgs::VisionGraspGetPose srv;
	srv.request.type = toRos(obj);

	bool result = client_get_pose_.call(srv);

	was_ros_service_succesfull_ = result;
	was_get_pose_succesfull_ = srv.response.success;

	if(!was_ros_service_succesfull_)
		std::cout << __PRETTY_FUNCTION__ << " could not call service" << std::endl;

	if(!was_get_pose_succesfull_)
		std::cout << __PRETTY_FUNCTION__ << " 3D pose detection was not succesfull" << std::endl;

	poseCam_ = toRw(srv.response.pose);

	return update_was_sucessfull();
}

void VisionSystem::prepare_pose(ObjectType obj)
{
	d7pc_msgs::VisionGraspPrepare srv;
	srv.request.type = toRos(obj);

	bool result = client_prepare_.call(srv);

	if(!result)
		std::cout << __PRETTY_FUNCTION__ << " could not call service" << std::endl;
}


bool VisionSystem::update_was_sucessfull() const
{
	if(!was_ros_service_succesfull_ || !was_get_pose_succesfull_)
		return false;
	return true;
}

rw::math::Transform3D<> VisionSystem::calc_camTobject() const
{
	rw::math::Transform3D<> poseCam = poseCam_;
	rw::math::Rotation3D<> rot(9.9989796615853233e-01, 1.3806455200754585e-02,
			-3.6659332808947893e-03, -1.3837249858978480e-02,
			9.9986802849085932e-01, -8.5121159615186570e-03,
			3.5479273344609730e-03, 8.5619738724021666e-03,
			9.9995705148523140e-01 );
	rot = rot.inverse();
	const rw::math::Transform3D<> camTpose(rot*poseCam.P(),rot*poseCam.R());
	return camTpose;
}


int VisionSystem::toRos(ObjectType object) const
{
	if (object == ObjectType::OuterShell)
		return d7pc_msgs::VisionGraspGetPoseRequest::typeOuterShell;
	if (object == ObjectType::InnerStructure)
		return d7pc_msgs::VisionGraspGetPoseRequest::typeInnerStructure;
	if (object == ObjectType::ScrewPart)
		return d7pc_msgs::VisionGraspGetPoseRequest::typeScrewPart;
	std::cout << __PRETTY_FUNCTION__ << " called with unknown type" << std::endl;
	return -1;
}

rw::math::Transform3D<> VisionSystem::toRw(const geometry_msgs::Pose& pose) const
{
  rw::math::Vector3D<> p(pose.position.x, pose.position.y, pose.position.z);
  rw::math::Quaternion<> q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  return rw::math::Transform3D<>(p, q);
}
