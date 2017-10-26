#ifndef D7PC_ROS_CARMEN_IIWA_INCLUDE_CARMEN_IIWA_INTERFACES_INSPECTION_SYSTEM_CPP_
#define D7PC_ROS_CARMEN_IIWA_INCLUDE_CARMEN_IIWA_INTERFACES_INSPECTION_SYSTEM_CPP_

#include <ros/ros.h>
#include <rw/math/Transform3D.hpp>

class InspectionSystem
{

public:

	enum class ObjectType {OuterShell, InnerStructure, ScrewPart, Pin};
	InspectionSystem();
	~InspectionSystem();

	bool update(ObjectType obj);
	bool was_update_sucessfull() const;
	rw::math::Transform3D<> calc_transform_tooTobject(ObjectType obj) const;
	double get_rotation_deg();

	rw::math::Transform3D<> get_toolTobject_ground_truth(ObjectType object) const;

private:
	int toRos(ObjectType obj) const;
	double get_axis_sign(ObjectType obj) const;
	double get_offset_deg(ObjectType obj) const;

private:
	ros::ServiceClient client_inspect_;
	double rotation_deg_;
	bool was_object_found_;
	bool was_inspection_succesfull_;
	bool was_ros_service_succesfull_;



};




#endif /* D7PC_ROS_CARMEN_IIWA_INCLUDE_CARMEN_IIWA_INTERFACES_INSPECTION_SYSTEM_CPP_ */
