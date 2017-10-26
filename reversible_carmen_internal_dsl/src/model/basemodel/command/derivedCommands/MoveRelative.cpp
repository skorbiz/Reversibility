/*
 * MoveRelative.cpp
 *
 *  Created on: Apr 20, 2015
 *      Author: josl
 */

#include "MoveRelative.hpp"


namespace dsl {

MoveRelative::MoveRelative(rw::math::Vector3D<> direction, dsl::common::WorkcellModel::frame frame, dsl::common::WorkcellModel wcmodel, std::shared_ptr<rwhw::UniversalRobotsRTLogging> urrt, std::shared_ptr<rwhw::URCallBackInterface> ur) :
	_urrt(urrt),
	_ur(ur),
	_direction(direction),
	_frame(frame),
	_wcModel(wcmodel),
	_speed(0.1),
	_acceleration(0.1)
{
}

MoveRelative::~MoveRelative()
{
}


void MoveRelative::waitForMovement(rw::math::Q qTo)
{
	while(true)
	{
		double norm = (qTo-_urrt->getLastData().qActual).norm2();
		ros::Duration(0.5).sleep();
		if(norm < 0.0001)
			break;
	}
}


void MoveRelative::execute()
{
	if(_qTo.size() == 0)
	{
		std::cerr << "Calculated qTo on runtime in MoveRelative" << std::endl;
		_qFrom = _urrt->getLastData().qActual;
		_qTo = calculateQto(_qFrom);
	}


	_ur->moveL(_qTo);
	waitForMovement(_qTo);
}

void MoveRelative::executeBackwards()
{
	assert(_qFrom.size() != 0);
	_ur->moveL(_qFrom);
	waitForMovement(_qFrom);
}

void MoveRelative::stateUpdate(dsl::State & state)
{
	_qFrom = state.getQ();

	if(_qFrom.size() == 0 )
		return;

	_qTo = calculateQto(_qFrom);
	state.update(_qTo);
}

void MoveRelative::stateUpdateSwapped(dsl::State & state)
{
	state.update(_qFrom);
}


std::string MoveRelative::getType()
{
	return "move relative";
}

rw::math::Q MoveRelative::calculateQto(rw::math::Q qfrom)
{
	rw::math::Q qCurrent = qfrom;
	assert(qCurrent.size() != 0 && "Attempted to use MoveRelative with unknown configuration");

	_wcModel.setRobotJointConfiguration(qCurrent);
	std::cerr << "ERROR IN MOVE RELATIVE.... CLASS NEEDS UPDATING" << std::endl;
	std::vector<rw::math::Q> qTargetSolutions;// = _wcModel.getNewConfigurationsForProjectedFrame(_frame, _direction);

	if(qTargetSolutions.size() == 0){std::cerr << "No solutions found in moveRelative" << std::endl;	exit(-1);}
	if(qTargetSolutions.size()  > 1){ std::cout << "Multiple solutions found in MoveRelative, firsts one selected" << std::endl;}

	rw::math::Q qTarget = qTargetSolutions[0];
	return qTarget;
}


} /* namespace dsl */
