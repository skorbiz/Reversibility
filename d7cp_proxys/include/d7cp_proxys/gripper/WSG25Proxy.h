/*
 * WSG25Proxy.h
 *
 *  Created on: Feb 7, 2017
 *      Author: jpb
 */

#ifndef D7PC_ROS_CARMEN_IIWA_INCLUDE_CARMEN_IIWA_GRIPPER_WSG25PROXY_H_
#define D7PC_ROS_CARMEN_IIWA_INCLUDE_CARMEN_IIWA_GRIPPER_WSG25PROXY_H_

#include <memory>

#include <ros/ros.h>
#include <roscpp/GetLoggers.h>
#include <roscpp/SetLoggerLevel.h>

#include <wsg_50_common/Status.h>
#include <wsg_50_common/Move.h>
#include <wsg_50_common/Incr.h>
#include <wsg_50_common/Conf.h>

namespace gripper {

class WSG25Proxy {
public:
	typedef std::shared_ptr<WSG25Proxy> Ptr;
	WSG25Proxy();
	virtual ~WSG25Proxy();

	/*
	 *	Returns the loggers for the gripper.
	 */
	virtual roscpp::GetLoggersResponse get_loggers(const roscpp::GetLoggersRequest & req);

	/*
	 * Grasps an object of a specific width at a specific velocity. Normally used setting a zero width object and a low velocity.
	 */
	virtual wsg_50_common::MoveResponse grasp(const wsg_50_common::MoveRequest & req);

	/*
	 * oves fingers to home position (maximum opening).
	 */
	virtual void homing();

	/*
	 * Moves fingers to an absolute position at a specific velocity. Deprecated by the creation of the next service.
	 */
	virtual wsg_50_common::MoveResponse move(const wsg_50_common::MoveRequest & req);

	/*
	 * Moves fingers to a specific distance in a specific direction (open/close) regarding the anterior position.
	 */
	virtual wsg_50_common::IncrResponse move_incrementally(const wsg_50_common::IncrRequest & req);

	/*
	 * Releases a grasped object opening the fingers to a indicated position.
	 */
	virtual wsg_50_common::MoveResponse release(const wsg_50_common::MoveRequest & req);

	/*
	 * Releases a grasped object opening the fingers to a indicated position.
	 */
	virtual wsg_50_common::ConfResponse set_acceleration(const wsg_50_common::ConfRequest & req);

	/*
	 * Set the force with the gripper grasp objects.
	 */
	virtual wsg_50_common::ConfResponse set_force(const wsg_50_common::ConfRequest & req);

	/*
	 *
	 */
	virtual roscpp::SetLoggerLevelResponse set_logger_level(const roscpp::SetLoggerLevelRequest & req);


	/*
	 * Publish the state of the gripper (fingers blocked, moving, position reached...) ,
	 * opening width, programmed acceleration and force at 1Hz (configurable on the main loop).
	 * (Where x can be tcp or can depeding on the node that we have executed.)
	 */
	virtual wsg_50_common::Status::ConstPtr get_status();

private:
	ros::NodeHandle nh_;
	ros::ServiceClient ark_client_;
	ros::ServiceClient get_loggers_client_;
	ros::ServiceClient grasp_client_;
	ros::ServiceClient homing_client_;
	ros::ServiceClient move_client_;
	ros::ServiceClient move_incrementally_client_;
	ros::ServiceClient release_client_;
	ros::ServiceClient set_acceleration_client_;
	ros::ServiceClient set_force_client_;
	ros::ServiceClient set_logger_level_client_;
	ros::ServiceClient stop_client_;
	ros::Subscriber get_status_subscriber_;
	wsg_50_common::Status::ConstPtr status_;
	void callback(wsg_50_common::Status::ConstPtr status);
};

} /* namespace gripper */

#endif /* D7PC_ROS_CARMEN_IIWA_INCLUDE_CARMEN_IIWA_GRIPPER_WSG25PROXY_H_ */
