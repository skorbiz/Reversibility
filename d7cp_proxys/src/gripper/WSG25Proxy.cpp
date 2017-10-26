/*
 * WSG25Proxy.cpp
 *
 *  Created on: Feb 7, 2017
 *      Author: jpb
 */

#include <gripper/WSG25Proxy.h>
#include <std_srvs/Empty.h>

namespace gripper {

const std::string deviceName = "wsg_50_driver";

WSG25Proxy::WSG25Proxy() : status_(nullptr)
{
	ark_client_ = nh_.serviceClient<std_srvs::Empty>(deviceName + "/ark");
	get_loggers_client_ = nh_.serviceClient<roscpp::GetLoggers>(deviceName + "/get_loggers");
	grasp_client_ = nh_.serviceClient<wsg_50_common::Move>(deviceName + "/grasp");
	homing_client_ = nh_.serviceClient<std_srvs::Empty>(deviceName + "/homing");
	move_client_ = nh_.serviceClient<wsg_50_common::Move>(deviceName + "/move");
	move_incrementally_client_ = nh_.serviceClient<wsg_50_common::Incr>(deviceName + "/move_incrementally");
	release_client_ = nh_.serviceClient<wsg_50_common::Move>(deviceName + "/release");
	set_acceleration_client_ = nh_.serviceClient<wsg_50_common::Conf>(deviceName + "/set_acceleration");
	set_force_client_ = nh_.serviceClient<wsg_50_common::Conf>(deviceName + "/set_force");
	set_logger_level_client_ = nh_.serviceClient<roscpp::SetLoggerLevel>(deviceName + "/set_logger_level");
	stop_client_ = nh_.serviceClient<std_srvs::Empty>(deviceName + "/stop");

	get_status_subscriber_ = nh_.subscribe(deviceName + "/status", 1, &WSG25Proxy::callback, this);
}

WSG25Proxy::~WSG25Proxy()
{
}

roscpp::GetLoggersResponse WSG25Proxy::get_loggers(const roscpp::GetLoggersRequest & req) {
	roscpp::GetLoggersResponse resp;
	get_loggers_client_.call(req,resp);
	return resp;
}

wsg_50_common::MoveResponse WSG25Proxy::grasp(const wsg_50_common::MoveRequest & req) {
	wsg_50_common::MoveResponse resp;
	grasp_client_.call(req,resp);
	return resp;
}

void WSG25Proxy::homing() {
	std_srvs::Empty srv;
	homing_client_.call(srv);
}

wsg_50_common::MoveResponse WSG25Proxy::move(const wsg_50_common::MoveRequest & req) {
	wsg_50_common::MoveResponse resp;
	move_client_.call(req,resp);
	return resp;
}

wsg_50_common::IncrResponse WSG25Proxy::move_incrementally(const wsg_50_common::IncrRequest & req) {
	wsg_50_common::IncrResponse resp;
	move_incrementally_client_.call(req,resp);
	return resp;
}

wsg_50_common::MoveResponse WSG25Proxy::release(const wsg_50_common::MoveRequest & req) {
	wsg_50_common::MoveResponse resp;
	release_client_.call(req,resp);
	return resp;
}

wsg_50_common::ConfResponse WSG25Proxy::set_acceleration(const wsg_50_common::ConfRequest & req) {
	wsg_50_common::ConfResponse resp;
	set_acceleration_client_.call(req,resp);
	return resp;
}

wsg_50_common::ConfResponse WSG25Proxy::set_force(const wsg_50_common::ConfRequest & req) {
	wsg_50_common::ConfResponse resp;
	set_force_client_.call(req,resp);
	return resp;
}

roscpp::SetLoggerLevelResponse WSG25Proxy::set_logger_level(const roscpp::SetLoggerLevelRequest & req) {
	roscpp::SetLoggerLevelResponse resp;
	set_logger_level_client_.call(req,resp);
	return resp;
}

wsg_50_common::Status::ConstPtr WSG25Proxy::get_status() {
	return status_;
}

void WSG25Proxy::callback(wsg_50_common::Status::ConstPtr status) {
	status_ =  status;
}


} /* namespace gripper */
