/*
 * vision_system.cpp
 *
 *  Created on: Dec 21, 2016
 *      Author: jpb
 */

#ifndef D7PC_ROS_CARMEN_IIWA_INCLUDE_CARMEN_IIWA_INTERFACES_VISION_SYSTEM_CPP_
#define D7PC_ROS_CARMEN_IIWA_INCLUDE_CARMEN_IIWA_INTERFACES_VISION_SYSTEM_CPP_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <rw/math/Transform3D.hpp>



class VisionSystem
{

public:
	enum class ObjectType { OuterShell,	InnerStructure,	ScrewPart };

	VisionSystem();
	~VisionSystem();

	bool update_pose(ObjectType obj);
	bool update_was_sucessfull() const;
	rw::math::Transform3D<> calc_camTobject() const;

	void prepare_pose(ObjectType obj);


private:
	int toRos(ObjectType obj) const;
	rw::math::Transform3D<> toRw(const geometry_msgs::Pose& pose) const;


private:
	ros::ServiceClient client_get_pose_;
	ros::ServiceClient client_prepare_;

	rw::math::Transform3D<> poseCam_;
	bool was_ros_service_succesfull_;
	bool was_get_pose_succesfull_;


};



#endif /* D7PC_ROS_CARMEN_IIWA_INCLUDE_CARMEN_IIWA_INTERFACES_VISION_SYSTEM_CPP_ */


