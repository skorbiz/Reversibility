/*
 * Action.hpp
 *
 *  Created on: Jan 21, 2015
 *      Author: josl
 */

#ifndef ACTION_HPP_
#define ACTION_HPP_


#include <iostream>
#include <rw/math.hpp>
#include <rw/trajectory.hpp>

#include <rwhw/universalrobots/UniversalRobotsRTLogging.hpp>
#include <rwhw/universalrobots/URCallBackInterface.hpp>
#include <rwhw/universalrobots/URDefinitions.hpp>

#include <../src/model/basemodel/command/CommandExecutable.hpp>

#include <ros/ros.h>

namespace dsl
{

class Action : public dsl::CommandExecutable
{

public:
	Action(std::shared_ptr<rwhw::UniversalRobotsRTLogging> urrt, std::shared_ptr<rwhw::URCallBackInterface> ur, rw::trajectory::TimedQPath qPath);
	virtual ~Action();

	virtual void execute();
	virtual void executeBackwards();
	void stateUpdate(dsl::State & state);
	void stateUpdateSwapped(dsl::State & state);
	virtual std::string getType();


private:
	void waitForMovement(rw::math::Q qTo);

	std::shared_ptr<rwhw::UniversalRobotsRTLogging> _urrt;
	std::shared_ptr<rwhw::URCallBackInterface> _ur;
	rw::trajectory::TimedQPath _qPath;
	double _blend;
	double _speed;
	double _acceleration;

};

} /* namespace dsl */

#endif /* ACTION_HPP_ */

