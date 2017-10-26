/*
 * MoveRelative.hpp
 *
 *  Created on: Apr 20, 2015
 *      Author: josl
 */

#ifndef SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_MOVERELATIVE_HPP_
#define SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_MOVERELATIVE_HPP_

#include <cassert>
#include <iostream>
#include <rw/rw.hpp>
#include <rw/math.hpp>
#include <RobotGeneralInterfaceProxy.hpp>

#include <rwhw/universalrobots/UniversalRobotsRTLogging.hpp>
#include <rwhw/universalrobots/URCallBackInterface.hpp>
#include <rwhw/universalrobots/URDefinitions.hpp>

#include <../src/model/basemodel/command/CommandExecutable.hpp>
#include <../src/model/basemodel/State.hpp>
#include <../src/common/WorkcellModel.hpp>

namespace dsl {

using namespace rw::math;

class MoveRelative  : public dsl::CommandExecutable
{

public:
	MoveRelative(rw::math::Vector3D<> direction, dsl::common::WorkcellModel::frame frame, dsl::common::WorkcellModel wcmodel, std::shared_ptr<rwhw::UniversalRobotsRTLogging> urrt, std::shared_ptr<rwhw::URCallBackInterface> ur);
	virtual ~MoveRelative();

	virtual void execute();
	virtual void executeBackwards();
	void stateUpdate(dsl::State & state);
	void stateUpdateSwapped(dsl::State & state);
	virtual std::string getType();

private:
	rw::math::Q calculateQto(rw::math::Q qfrom);

private:
	void waitForMovement(rw::math::Q qTo);

	std::shared_ptr<rwhw::UniversalRobotsRTLogging> _urrt;
	std::shared_ptr<rwhw::URCallBackInterface> _ur;
	rw::math::Q _qFrom;
	rw::math::Q _qTo;

	rw::math::Vector3D<> _direction;
	dsl::common::WorkcellModel::frame _frame;
	dsl::common::WorkcellModel _wcModel;

	double _speed;
	double _acceleration;

};

} /* namespace dsl */

#endif /* SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_MOVERELATIVE_HPP_ */
