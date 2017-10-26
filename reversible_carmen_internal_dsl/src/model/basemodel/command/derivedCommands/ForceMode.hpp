/*
 * ForceMode.hpp
 *
 *  Created on: Jan 20, 2015
 *      Author: josl
 */

#ifndef FORCEMODE_HPP_
#define FORCEMODE_HPP_

#include <rwhw/universalrobots/UniversalRobotsRTLogging.hpp>
#include <rwhw/universalrobots/URCallBackInterface.hpp>
#include <rwhw/universalrobots/URDefinitions.hpp>

#include <../src/model/basemodel/argument/quantities/ForceModeArgument.hpp>
#include <../src/model/basemodel/command/CommandExecutable.hpp>

namespace dsl
{

class ForceMode : public dsl::CommandExecutable
{

public:
	ForceMode(std::shared_ptr<rwhw::UniversalRobotsRTLogging> urrt, std::shared_ptr<rwhw::URCallBackInterface> ur, ForceModeArgument fma_forward, ForceModeArgument fma_backward);
	virtual ~ForceMode();

	virtual void execute();
	virtual void executeBackwards();
	virtual std::string getType();

	void stateUpdate(dsl::State & state);
	void stateUpdateSwapped(dsl::State & state);


private:
	void executeFMA(ForceModeArgument fma);

	std::shared_ptr<rwhw::UniversalRobotsRTLogging> _urrt;
	std::shared_ptr<rwhw::URCallBackInterface> _ur;

	ForceModeArgument _fma_forward;
	ForceModeArgument _fma_backward;
};

} /* namespace dsl */

#endif /* FORCEMODE_HPP_ */
