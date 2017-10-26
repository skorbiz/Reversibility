#include <InspectionSystem.h>

#include <limits>

#include <d7pc_msgs/VisionInspection.h>

#include <rw/math/RPY.hpp>
#include <rw/math/Vector3D.hpp>

InspectionSystem::InspectionSystem() :
	rotation_deg_(0),
	was_object_found_(false),
	was_inspection_succesfull_(false),
	was_ros_service_succesfull_(false)

{
	ros::NodeHandle nh;
	client_inspect_ = nh.serviceClient<d7pc_msgs::VisionInspection>("/d7pc_2d_inspection/inspect");
}

InspectionSystem::~InspectionSystem()
{
}

bool InspectionSystem::update(ObjectType obj)
{
	d7pc_msgs::VisionInspection srv;
	srv.request.objectId = toRos(obj);

	bool result = client_inspect_.call(srv);

	was_ros_service_succesfull_ = result;
	was_inspection_succesfull_ = srv.response.inspection_ok;
	was_object_found_ = srv.response.object_found;

	if(!was_ros_service_succesfull_)
		std::cout << __PRETTY_FUNCTION__ << " could not call service" << std::endl;

	if(!was_inspection_succesfull_)
		std::cout << __PRETTY_FUNCTION__ << " Inspection was not succesfull" << std::endl;

	if(!was_object_found_)
		std::cout << __PRETTY_FUNCTION__ << " Object was not found" << std::endl;

	rotation_deg_ = srv.response.rotation_deg;

	return was_update_sucessfull();
}


bool InspectionSystem::was_update_sucessfull() const
{
	if(!was_ros_service_succesfull_ || !was_inspection_succesfull_ || !was_object_found_)
		return false;
	return true;
}

rw::math::Transform3D<> InspectionSystem::calc_transform_tooTobject(ObjectType obj) const
{
	double offset = get_offset_deg(obj);
	double sign = get_axis_sign(obj);

	double deg2rad = M_PI/180.0;
	double angle_deg = sign*rotation_deg_ + offset;
	double angle_rad = angle_deg * deg2rad;

	std::cout << "deg: " << angle_deg << std::endl;

	rw::math::RPY<> rpy(0, angle_rad, 0);
	rw::math::Vector3D<> vec(0,0,0);
	rw::math::Transform3D<> object_gtTobject_actual(vec, rpy.toRotation3D());
	rw::math::Transform3D<> toolTobject_gt = get_toolTobject_ground_truth(obj);
	return toolTobject_gt * object_gtTobject_actual;
}

double InspectionSystem::get_rotation_deg()
{
	return rotation_deg_;
}



int InspectionSystem::toRos(ObjectType object) const
{
	if(object == ObjectType::OuterShell)
		return d7pc_msgs::VisionInspection::Request::objectIdOuterShell;
	if(object == ObjectType::InnerStructure)
		return d7pc_msgs::VisionInspection::Request::objectIdInnerStructure;
	if(object == ObjectType::ScrewPart)
		return d7pc_msgs::VisionInspection::Request::objectIdScrewPart;
	if(object == ObjectType::Pin)
		return d7pc_msgs::VisionInspection::Request::objectIdPin;
	std::cout << __PRETTY_FUNCTION__ << " called with unknown type" << std::endl;
	return -1;
}

double InspectionSystem::get_axis_sign(ObjectType object) const
{
	if(object == ObjectType::OuterShell)
		return 1;
	if(object == ObjectType::InnerStructure)
		return -1;
	if(object == ObjectType::ScrewPart)
		return 1;
	if(object == ObjectType::Pin)
		return 1;
	std::cout << __PRETTY_FUNCTION__ << " called with unknown type" << std::endl;
	return std::numeric_limits<double>::quiet_NaN();
}

double InspectionSystem::get_offset_deg(ObjectType object) const
{
	if(object == ObjectType::OuterShell)
		return -35; //-37 	//+ is counter clockwise
	if(object == ObjectType::InnerStructure)
		return 90;
	if(object == ObjectType::ScrewPart)
		return -90;
	if(object == ObjectType::Pin)
		return 90;
	std::cout << __PRETTY_FUNCTION__ << " called with unknown type" << std::endl;
	return std::numeric_limits<double>::quiet_NaN();
}

rw::math::Transform3D<> InspectionSystem::get_toolTobject_ground_truth(ObjectType object) const
{

	double deg2rad = M_PI/180.0;
	rw::math::Vector3D<> pos(0, 0, 0);
	rw::math::RPY<> rpy(0, 0, 0);
	if(object == ObjectType::OuterShell)
	{
		pos = rw::math::Vector3D<>(0, 0.023, 0);
		rpy = rw::math::RPY<>(180*deg2rad, 0, 0);
	}
	else if(object == ObjectType::InnerStructure)
	{
		pos = rw::math::Vector3D<>(0, 0.012, 0);
		rpy = rw::math::RPY<>(180*deg2rad, 0, 0);
	}
	else if(object == ObjectType::ScrewPart)
	{
		pos = rw::math::Vector3D<>(0, -0.0035, 0);
		rpy = rw::math::RPY<>(180*deg2rad, 0, 0);
	}
	else if(object == ObjectType::Pin)
	{
		pos = rw::math::Vector3D<>(0.015, 0, 0);
		rpy = rw::math::RPY<>( 90*deg2rad, 0, 0);
	}
	else
		std::cout << __PRETTY_FUNCTION__ << " called with unknown type" << std::endl;

	rw::math::Transform3D<> toolTobjectGT(pos, rpy.toRotation3D());
	return toolTobjectGT;
}
