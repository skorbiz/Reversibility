/*
 * Move.hpp
 *
 *  Created on: Nov 6, 2014
 *      Author: josl
 */

#ifndef MOVE_HPP_
#define MOVE_HPP_

#include <iostream>
#include <rw/math.hpp>

#include <rwhw/universalrobots/UniversalRobotsRTLogging.hpp>
#include <rwhw/universalrobots/URCallBackInterface.hpp>
#include <rwhw/universalrobots/URDefinitions.hpp>

#include <../src/model/basemodel/command/CommandExecutable.hpp>
#include <../src/model/basemodel/State.hpp>

namespace dsl
{

class Move : public dsl::CommandExecutable
{

public:
	Move( std::shared_ptr<rwhw::UniversalRobotsRTLogging> urrt, std::shared_ptr<rwhw::URCallBackInterface> ur, rw::math::Q qTo);
	virtual ~Move();
	virtual void execute();
	virtual void executeBackwards();
	void stateUpdate(dsl::State & state);
	void stateUpdateSwapped(dsl::State & state);
	virtual std::string getArgumentForward() const;
	virtual std::string getArgumentBackward() const;
	virtual std::string getType();

private:
	void waitForMovement(rw::math::Q qTo);

	std::shared_ptr<rwhw::UniversalRobotsRTLogging> urrt;
	std::shared_ptr<rwhw::URCallBackInterface> ur;


	rw::math::Q qFrom;
	rw::math::Q qTo;
	double speed;
	double acceleration;

};

} /* namespace dsl */

#endif /* MOVE_HPP_ */

