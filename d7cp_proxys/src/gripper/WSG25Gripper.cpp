/*
 * WSG25Gripper.cpp
 *
 *  Created on: Feb 7, 2017
 *      Author: jpb
 */

#include <gripper/WSG25Gripper.h>

namespace gripper {

WSG25Gripper::WSG25Gripper(WSG25Proxy::Ptr wsg25proxy) : wsg25_(wsg25proxy){
}


bool WSG25Gripper::move(double width, double speed) {
	wsg_50_common::MoveRequest release_request;
	release_request.width = width;
	release_request.speed = speed;
	wsg25_->move(release_request);
	return true;
}

bool WSG25Gripper::open(double speed) {
	wsg_50_common::MoveRequest release_request;
	release_request.width = 64;
	release_request.speed = speed;
	wsg25_->move(release_request);
	return true;
}

bool WSG25Gripper::close(double speed) {
	wsg_50_common::MoveRequest graspReq;
	graspReq.width = 0;
	graspReq.speed = speed;
	wsg25_->move(graspReq);
	return true;
}


bool WSG25Gripper::grasp(double width, double speed)
{
	wsg_50_common::MoveRequest graspReq;
	graspReq.width = width;
	graspReq.speed = speed;
	wsg25_->grasp(graspReq);
	return true;
}

bool WSG25Gripper::release(double width, double speed) {

	wsg_50_common::MoveRequest release_request;
	release_request.width = width;
	release_request.speed = speed;
	wsg25_->release(release_request);
	return true;
}

bool WSG25Gripper::set_force(double force)
{
	wsg_50_common::ConfRequest set_force_request;
	set_force_request.val = force;
	wsg25_->set_force(set_force_request);
	return true;
}


bool WSG25Gripper::block_while_moving()
{
	while( not atRest());
	ros::Duration(0.5).sleep();
	return true;
}

bool WSG25Gripper::atRest() {
	return ( this->getSpeed() < 0.001 );
}


double  WSG25Gripper::getWidth() {
	if(wsg25_->get_status() == nullptr) {
		ROS_ERROR("WSG25Gripper::getWidth() wsg_25_status has not been published");
		return -1;
	}
	return wsg25_->get_status()->width;
}

double  WSG25Gripper::getSpeed() {
	if(wsg25_->get_status() == nullptr) {
		ROS_ERROR("WSG25Gripper::getSpeed() wsg_25_status has not been published");
		return -1;
	}
	return wsg25_->get_status()->speed;
}

double  WSG25Gripper::getForce() {
	if(wsg25_->get_status() == nullptr) {
		ROS_ERROR("WSG25Gripper::getForce() wsg_25_status has not been published");
		return -1;
	}
	return wsg25_->get_status()->force;
}


} /* namespace gripper */
