/*
 * TestJoy.hpp
 *
 *  Created on: Jun 3, 2016
 *      Author: josl
 */

#ifndef REVERSIBLE_DSL_SRC_PROGRAMS_TESTJOY_HPP_
#define REVERSIBLE_DSL_SRC_PROGRAMS_TESTJOY_HPP_

#include <rwhw/universalrobots/UniversalRobotsRTLogging.hpp>
#include <rwhw/universalrobots/URCallBackInterface.hpp>
#include <rwhw/universalrobots/URDefinitions.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <../src/common/WorkcellModel.hpp>

namespace dsl
{

class TestJoy
{

public:
	TestJoy();
	virtual ~TestJoy();

	void callbackJoy(const sensor_msgs::Joy::ConstPtr& msg);
	void connectToUR();
	void doServo();
	rw::math::Q calculateQto(rw::math::Q qfrom);
	rw::math::Transform3D<> calculateMoveDist();
	rw::math::Transform3D<> calculateCenterDist();
	void updataUserScale();


	std::shared_ptr<rwhw::UniversalRobotsRTLogging> _urrt;
	std::shared_ptr<rwhw::URCallBackInterface> _ur;


	std::shared_ptr<ros::NodeHandle> _nodeHnd;
	ros::Subscriber _subJoystik;

	dsl::common::WorkcellModel _wcModel;

	double _userScale;

	//Buttons and axis mapping
	double _leftStick;
	double _leftStickUD;
	double _leftStickLR;
	double _rightStick;
	double _rightStickUD;
	double _rightStickLR;

	double _leftBumber;
	double _rightBumber;

	double _aButton;
	double _bButton;
	double _xButton;
	double _yButton;




};

} /* namespace dsl */

#endif /* REVERSIBLE_DSL_SRC_PROGRAMS_TESTJOY_HPP_ */
