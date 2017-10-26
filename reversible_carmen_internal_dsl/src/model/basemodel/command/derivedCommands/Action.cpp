/*
 *
 *  Created on: Jan 21, 2015
 *      Author: josl
 */

#include "Action.hpp"

namespace dsl
{

Action::Action(std::shared_ptr<rwhw::UniversalRobotsRTLogging> urrt, std::shared_ptr<rwhw::URCallBackInterface> ur, rw::trajectory::TimedQPath qPath) :
		_urrt(urrt),
		_ur(ur),
		_qPath(qPath),
		_blend(0.0005),
//		_speed(15.00),	_acceleration(2.5)		// Very fast (100 test)
//		_speed( 2.00),	_acceleration(0.5)		// fast		 (for tested programs)
		_speed( 0.40),	_acceleration(0.1)
//		_speed( 0.05),	_acceleration(0.1)		//Slow
//		_speed( 0.01),	_acceleration(0.1)		//Painfully slow

{
}

Action::~Action()
{
}

void Action::execute()
{
	double epsilonQ = 0.030;

	for(unsigned int i = 0; i < _qPath.size(); i++)
	{
		rw::math::Q qNext = _qPath[i].getValue();

		while(true)
		{
			rw::math::Q qCurrent = _urrt->getLastData().qActual;

			// Calculate new q
			assert(qNext.size() == 6);
			assert(qCurrent.size() == 6);
			double diffToEnd =  (qNext-qCurrent).norm2();
			rw::math::Q deltaQ = (qNext-qCurrent)/diffToEnd*epsilonQ;
			rw::math::Q qTarget = qCurrent + deltaQ;

			// Qend reached
			if(epsilonQ > diffToEnd)
				break;

			// Servo to qTarget
			_ur->servo(qTarget);	//_urp->servoQ(qTarget);
		}
	}

	rw::math::Q qEnd = _qPath[_qPath.size()-1].getValue();
	_ur->servo(qEnd);
	ros::Duration(0.5).sleep();
}




//void Action::execute()
//{
////	gen::dCommand << "executing Action" << std::endl;
////	bool docheck = true;
//	for(unsigned int i = 0; i < _qPath.size(); i++)
//	{
////		std::cout << "Action :: step: " << i << std::endl;
//		_ur->moveL(_qPath[i].getValue(),_speed,_acceleration);
//		waitForMovement(_qPath[i].getValue());
//
////		if(docheck)
////		{
////			std::cout << "step?" << std::endl;
////			std::string input;
////			std::cin >> input;
////			std::cout << "i: " << i << " / " <<  _qPath.size() << "   input: " << input << std::endl;
////
////			if(input == "f")
////				docheck = false;
////
////			if(input == "b")
////				i = i-2;
////
////			if(input == "e")
////				return;
////		}
//
//	}
//}


void Action::executeBackwards()
{
//	gen::dCommand << "executed Action Backwards" << std::endl;
//	int last = _qPath.size() -1;
//	_RGIP->moveJ(_qPath[last].getValue(), _speed, _acceleration);

	for(unsigned int i = _qPath.size()-1; i > 0; i--)
	{
		_ur->moveJ(_qPath[i].getValue(),_speed,_acceleration);
		waitForMovement(_qPath[i].getValue());
	}
//		_RBIP->appendQ(_qPath[i-1].getValue(), _speed, _acceleration,  _blend);
//	_RBIP->execute();
}

void Action::waitForMovement(rw::math::Q qTo)
{
	while(true)
	{
		double norm = (qTo-_urrt->getLastData().qActual).norm2();
		ros::Duration(0.1).sleep();
		std::cout << norm << std::endl;
		if(norm < 0.01)
			break;
	}
}


void Action::stateUpdate(dsl::State & state)
{
	int last = _qPath.size() -1;
	state.update(_qPath[last].getValue());
}

void Action::stateUpdateSwapped(dsl::State & state)
{
	state.update(_qPath[0].getValue());
}

std::string Action::getType()
{
	return "action";
}


} /* namespace dsl */
