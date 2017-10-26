/*
 * RobworkInterface.h
 *
 *  Created on: Mar 9, 2017
 *      Author: josl
 */

#ifndef D7CP_PROXYS_INCLUDE_D7CP_PROXYS_ROBOT_MOVEMENT_INTERFACE_ROBWORKINTERFACE_H_
#define D7CP_PROXYS_INCLUDE_D7CP_PROXYS_ROBOT_MOVEMENT_INTERFACE_ROBWORKINTERFACE_H_

#include <memory>
#include "RosProxy.h"
#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>


namespace robot_movement_interface
{

class RobworkInterface
{

public:
	RobworkInterface(std::shared_ptr<RosProxy> ros_proxy);
	virtual ~RobworkInterface();


	void movePtp(rw::math::Q q);
	void movePtp(std::vector<rw::math::Q> path);
	void moveLin(rw::math::Transform3D<> baseTtool);
	void moveLinRedundency(rw::math::Transform3D<> baseTtool, double redundency);
    void moveComplience(rw::math::Transform3D<> baseTtool);
	rw::math::Q getQ();
	rw::math::Transform3D<> getBaseTtool();

	void stop();
	bool is_moving();

	bool is_at(rw::math::Q q, double eps = 0.01);
	bool is_at(rw::math::Transform3D<> baseTtool, double eps = 0.01);

	rw::math::Q toRw(sensor_msgs::JointState msg);
	rw::math::Transform3D<> toRw(geometry_msgs::Wrench msg);
	rw::math::Transform3D<> toRw(robot_movement_interface::EulerFrame);

	std::string toString(rw::math::Q);
	std::string toString(rw::math::Transform3D<>);

private:
	std::string callCommand(const std::string & command, const std::string & param);

	std::shared_ptr<RosProxy> proxy;


};

} /* namespace robot_movement_interface */

#endif /* D7CP_PROXYS_INCLUDE_D7CP_PROXYS_ROBOT_MOVEMENT_INTERFACE_ROBWORKINTERFACE_H_ */


