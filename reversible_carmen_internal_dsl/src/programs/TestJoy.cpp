/*
 * TestJoy.cpp
 *
 *  Created on: Jun 3, 2016
 *      Author: josl
 */

#include "TestJoy.hpp"

namespace dsl {

TestJoy::TestJoy() :
		_userScale(1)
{
	std::cout << "Test joy start" << std::endl;

	_nodeHnd = std::make_shared<ros::NodeHandle>();
	_subJoystik = _nodeHnd->subscribe("/joy", 1, &TestJoy::callbackJoy, this);

	connectToUR();

	while(true)
		doServo();


	std::cout << "test joy end" << std::endl;

}

TestJoy::~TestJoy()
{
}

void TestJoy::doServo()
{
//	std::cout << "servo loop" << std::endl;
	const double epsilonQ = 0.030;

	rwhw::URRTData data = _urrt->getLastData();			//	 qC = _urp->getQActual();
	rw::math::Q qCurrent = data.qActual;

//	double diffToEnd =  (_qEnd-_qCurrent).norm2();
//	rw::math::Q deltaQ = (_qEnd-_qCurrent)/diffToEnd*epsilonQ;
//	rw::math::Q qTarget = _qCurrent + deltaQ;

	rw::math::Q qTarget = calculateQto(qCurrent);

	// Servo to qTarget
	_ur->servo(qTarget);	//_urp->servoQ(qTarget);

	ros::Duration(0.01).sleep();
}

rw::math::Q TestJoy::calculateQto(rw::math::Q qfrom)
{
	rw::math::Q qCurrent = qfrom;
	assert(qCurrent.size() != 0 && "Attempted to use MoveRelative with unknown configuration");

	_wcModel.setRobotJointConfiguration(qCurrent);

	dsl::common::WorkcellModel::frame frameBase = dsl::common::WorkcellModel::frame::base;
	dsl::common::WorkcellModel::frame frameTool = dsl::common::WorkcellModel::frame::toolmount;


	rw::math::Transform3D<>  transform = _wcModel.getTransformParentTChild(frameBase,frameTool);
	rw::math::Transform3D<> tChange = calculateMoveDist();
	transform = transform * tChange;

	if(_bButton)
	{
		std::cout << "going center" << std::endl;
		transform = calculateCenterDist();
	}

	std::vector<rw::math::Q> qTargetSolutions = _wcModel.getNewConfigurationsForTransform(transform);

	if(qTargetSolutions.size() == 0){std::cerr << "No solutions found in moveRelative" << std::endl;	exit(-1);}
	if(qTargetSolutions.size()  > 1){ std::cout << "Multiple solutions found in MoveRelative, firsts one selected" << std::endl;}

	rw::math::Q qTarget = qTargetSolutions[0];
	return qTarget;
}


rw::math::Transform3D<> TestJoy::calculateMoveDist()
{
	const double scaleP = 50.0;
	const double scaleR = 10.0;

	rw::math::Vector3D<double> vec;
	if(!_rightStick)
		vec = rw::math::Vector3D<double>(_leftStickUD,-_leftStickLR,0);
	else
		vec = rw::math::Vector3D<double>(0,0,-_leftStickUD);
	vec = vec/scaleP*_userScale;

	rw::math::RPY<double> ret;
	if(!_leftStick)
		ret = rw::math::RPY<double>(-_rightStickLR/scaleR*_userScale,_rightStickUD/scaleR*_userScale,0);
	else
		ret = rw::math::RPY<double>(0,0,_rightStickLR/scaleR*_userScale);

	rw::math::Transform3D<> t(vec, ret.toRotation3D());

	return t;
}

rw::math::Transform3D<> TestJoy::calculateCenterDist()
{
	dsl::common::WorkcellModel::frame frameBase = dsl::common::WorkcellModel::frame::base;
	dsl::common::WorkcellModel::frame frameTool = dsl::common::WorkcellModel::frame::toolmount;
	rw::math::Transform3D<>  transform = _wcModel.getTransformParentTChild(frameBase,frameTool);

	rw::math::Vector3D<double> pos = transform.P();
	rw::math::Rotation3D<double> rot = transform.R();

	rw::math::RPY<double> rpy(rot);

	for(int i = 0; i < 3; i++)
	{
		double a = rpy[i]*rw::math::Rad2Deg;
		a = std::round(a/90.0)*90.0;
		rpy[i] = a * rw::math::Deg2Rad;
	}

	rw::math::Transform3D<> t(pos,rpy.toRotation3D());

	return t;
}

void TestJoy::updataUserScale()
{
	if(_rightBumber)
		_userScale += 0.1;

	if(_leftBumber)
		_userScale -= 0.1;

	if(_userScale < 0.0)
		_userScale = 0.0;

	if(_userScale > 1.0)
		_userScale = 1.0;

	std::cout << "user scale: " << _userScale << std::endl;
}



void TestJoy::callbackJoy(const sensor_msgs::Joy::ConstPtr& msg)
{

	std::vector<float> axes = msg->axes;
	std::vector<int32_t> buttons = msg->buttons;

	std::cout << "axies: ";
	for(int i = 0; i < axes.size(); i++)
		std::cout << i << "=" << axes[i] << "   ";
	std::cout << std::endl;

	std::cout << "Buttons: ";
	for(int i = 0; i < buttons.size(); i++)
		std::cout << i << "=" << buttons[i] << "   ";
	std::cout << std::endl;

	_leftStick = buttons[13];
	_leftStickUD = axes[1];
	_leftStickLR = axes[0];
	_rightStick = buttons[14];
	_rightStickUD = axes[3];
	_rightStickLR = axes[2];

	_leftBumber = buttons[6];
    _rightBumber = buttons[7];

	_aButton = buttons[0];
	_bButton = buttons[1];
	_xButton = buttons[2];
	_yButton = buttons[3];


	std::cout << "left stick  " << _leftStick << std::endl;
	std::cout << "left stickUD" << _leftStickUD << std::endl;
	std::cout << "left stickLR" << _leftStickLR << std::endl;
	std::cout << "right stick  " << _rightStick << std::endl;
	std::cout << "right stickUD" << _rightStickUD << std::endl;
	std::cout << "right stickLR" << _rightStickLR << std::endl;

	std::cout << "left bumber" << _leftBumber << std::endl;
	std::cout << "right bumber" << _rightBumber << std::endl;

	std::cout << "a " << _aButton << std::endl;
	std::cout << "b " << _bButton << std::endl;
	std::cout << "x " << _xButton << std::endl;
	std::cout << "y " << _yButton << std::endl;

	updataUserScale();
}

void TestJoy::connectToUR()
{
	unsigned int urrt_port = 30003;
	unsigned int ur_port = 30001;
	std::string device_ip = "192.168.1.250";
	std::string callback_ip = "192.168.1.240";
	unsigned int numeric_callback_port = 33333;

	_urrt = std::make_shared<rwhw::UniversalRobotsRTLogging>(universalrobots::CB31);
	_ur = std::make_shared<rwhw::URCallBackInterface>(universalrobots::CB31);

	std::cout << "Connecting to UR interface" << std::endl;
	try
	{
		_urrt->connect(device_ip, urrt_port);
	}
	catch (rw::common::Exception& exp)
	{
		std::cout << "Could not connect to urrt:" << exp.what() << std::endl;
	}

	try
	{
		_ur->connect(device_ip, ur_port);
	}
	catch (rw::common::Exception& exp)
	{
		std::cout << "Could not connect to urrt:" << exp.what() << std::endl;
	}

	_ur->startCommunication(callback_ip, numeric_callback_port);
	_urrt->start();

	std::cout << "waiting for UR connection" << std::endl;
	while(!_urrt->hasData())							//	rw::math::Q qC(7);
		std::cout << ".";								//	while(qC.size() != 6)


	std::cout << "Connected to UR interface" << std::endl;

	std::cout << "Opening Intelligent Move dataset" << std::endl;
}


} /* namespace dsl */
