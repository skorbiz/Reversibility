/*
 * PositionCartesianComparison.cpp
 *
 *  Created on: Jan 8, 2015
 *      Author: josl
 */

#include "PositionCartesianComparison.hpp"

namespace dsl {

using namespace rw::math;

PositionCartesianComparison::PositionCartesianComparison(const dsl::JointConfiguration & q, double epsilon, std::shared_ptr<RobotGeneralInterfaceProxy> RGIP) :
		_RGIP(RGIP),
		_qExpected(q.get()),
		_epsilon(epsilon)
{
	dco.setPrefix("[Distance error]");
	dco.setColor(dsl::color::GREEN);
}

PositionCartesianComparison::PositionCartesianComparison(rw::math::Q q, double epsilon, std::shared_ptr<RobotGeneralInterfaceProxy> RGIP) :
		_RGIP(RGIP),
		_qExpected(q),
		_epsilon(epsilon)
{
	dco.setPrefix("[Distance error]");
	dco.setColor(dsl::color::GREEN);
}


PositionCartesianComparison::~PositionCartesianComparison()
{
}


bool PositionCartesianComparison::evaluate()
{

	dsl::common::WorkcellModel wc;
	dsl::common::WorkcellModel::frame frameBase = dsl::common::WorkcellModel::frame::base;
	dsl::common::WorkcellModel::frame frameSelected = dsl::common::WorkcellModel::frame::toolmount;

	rw::math::Transform3D<> base2Target;
	rw::math::Transform3D<> base2Actual;
	rw::math::Transform3D<> target2Actual;

	rw::math::Q target = _qExpected;
	rw::math::Q actual = _RGIP->getQ();

	wc.setRobotJointConfiguration(target);
	base2Target = wc.getTransformParentTChild(frameBase,frameSelected);

	wc.setRobotJointConfiguration(actual);
	base2Actual = wc.getTransformParentTChild(frameBase, frameSelected);

	target2Actual = inverse(base2Target) * base2Actual;

	std::cout << target2Actual << std::endl;
	return true;




	rw::math::Q qActual = _RGIP->getQ();
	rw::math::Q qDiff = _qExpected - qActual;
	double error = qDiff.norm2();

	errorVisualizer(error);

	if(error < _epsilon)
		return true;

//	dco << "Error occurred in distance comparison" << std::endl;
//	dco << std::setprecision(5) << std::fixed << "Q expected: " << _qExpected << "\t in degrees: " << _qExpected * 180.0 / rw::math::Pi << std::endl;
//	dco << std::setprecision(5) << std::fixed << "Q actual:   " << qActual 	<< "\t in degrees: " << qActual * 180.0 / rw::math::Pi << std::endl;
//	dco << std::setprecision(5) << std::fixed << "Q diff;     " << qDiff 		<< "\t in degrees: " << qDiff * 180.0 / rw::math::Pi << std::endl;
//	dco << "Epsilon was: " << _epsilon << std::endl;
//	dco << "Error was:   " << error << std::endl;
	return false;
}

void PositionCartesianComparison::stateUpdate(dsl::State const& state)
{
	if(_qExpected.size() == 0)
		_qExpected = state.getQ();

	if(_qExpected.size() == 0)
	{
		std::cerr << "Obs tried to make joint position condition from unkown joint state - this is not allowed" << std::endl;
		exit(-1);
	}
}

void PositionCartesianComparison::addArgument(const dsl::JointConfiguration & q)
{
	_qExpected = q.get();
}

void PositionCartesianComparison::errorVisualizer(double error)
{
	//Robot return precsion: 	0.000 - 0.00099		(Typical error: 0.00019
	//small error				0.001 -	0.0099 		(Hardly detectable by humans, but clearly error: 0.005)
	//medium error: 			0.01  - 0.1			(emergency stop: 0.1)

	if(error > 0.001)	dco << dsl::color::YELLOW;										//Hardly visiable
	if(error > 0.01)	dco << dsl::color::RED;											//Slightly visiable - enough for most cases
	if(error > 0.02)	dco << dsl::color::WHITE;
	if(error > 0.03)	dco << dsl::color::BOLDWHITE;									//Clearly visiable
	if(error > 0.04)	dco << dsl::color::COLORRESET<< dsl::color::CYAN;
	if(error > 0.05)	dco << dsl::color::BOLDCYAN;
	if(error > 0.06)	dco << dsl::color::COLORRESET<< dsl::color::MAGENTA;
	if(error > 0.07)	dco << dsl::color::BOLDMAGENTA;									//Close to 1 cm
	if(error > 0.08)	dco << dsl::color::COLORRESET<< dsl::color::BLUE;		//Close to emergency stop

	dco << "Error was:   " << error << std::endl;
}


} /* namespace dsl */
