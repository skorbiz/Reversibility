/*
 * PositionJointComparison.cpp
 *
 *  Created on: Jan 8, 2015
 *      Author: josl
 */

#include "PositionJointComparison.hpp"

namespace dsl {

PositionJointComparison::PositionJointComparison(const dsl::JointConfiguration & q, double epsilon,std::shared_ptr<rwhw::UniversalRobotsRTLogging> urrt) :
		_urrt(urrt),
		_qExpected(q.get()),
		_epsilon(epsilon)
{
	dco.setPrefix("[Distance error]");
	dco.setColor(dsl::color::GREEN);
}

PositionJointComparison::PositionJointComparison(rw::math::Q q, double epsilon, std::shared_ptr<rwhw::UniversalRobotsRTLogging> urrt) :
		_urrt(urrt),
		_qExpected(q),
		_epsilon(epsilon)
{
	dco.setPrefix("[Distance error]");
	dco.setColor(dsl::color::GREEN);
}


PositionJointComparison::~PositionJointComparison()
{
}

bool PositionJointComparison::evaluate()
{
	rw::math::Q qActual = _urrt->getLastData().qActual;
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

void PositionJointComparison::stateUpdate(dsl::State const& state)
{
	if(_qExpected.size() == 0)
		_qExpected = state.getQ();

	if(_qExpected.size() == 0)
	{
		std::cerr << "Obs tried to make joint position condition from unkown joint state - this is not allowed" << std::endl;
		exit(-1);
	}
}

void PositionJointComparison::addArgument(const dsl::JointConfiguration & q)
{
	_qExpected = q.get();
}

void PositionJointComparison::errorVisualizer(double error)
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
