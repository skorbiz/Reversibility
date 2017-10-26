/*
 * IntelligentActionModelInterface.hpp
 *
 *  Created on: Jun 27, 2016
 *      Author: josl
 */

#ifndef REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_INTELLIGENTACTIONMODELINTERFACE_HPP_
#define REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_INTELLIGENTACTIONMODELINTERFACE_HPP_

#include <cassert>

#include <ros/ros.h>
#include <ros/package.h>

#include <rw/RobWork.hpp>
#include <rw/math.hpp>
#include <rw/trajectory/Path.hpp>

#include <rwhw/universalrobots/UniversalRobotsRTLogging.hpp>
#include <rwhw/universalrobots/URCallBackInterface.hpp>
#include <rwhw/universalrobots/URDefinitions.hpp>

#include <LearningProxy.hpp>

#include "IntelligentTrajectory.hpp"
#include <../src/model/basemodel/command/CommandExecutable.hpp>
#include <../src/model/basemodel/command/derivedCommands/ActionIKSolver.hpp>

#include "RecordDataset.hpp"

namespace dsl {
namespace intelligentmove {

class IntelligentActionModelInterface : public dsl::CommandExecutable{
public:
	IntelligentActionModelInterface(
			std::shared_ptr<rwhw::URCallBackInterface> urc,
			std::shared_ptr<rwhw::UniversalRobotsRTLogging> urrt,
			std::shared_ptr<RecordDataset> set,
			std::string filepathDataset,
			std::string filepathTrajectory,
			std::string frameTool,
			std::string frameFixture,
			rw::math::Q qInit
	);
	virtual ~IntelligentActionModelInterface();

	virtual void execute();
	virtual void executeBackwards();
	void stateUpdate(dsl::State & state);
	void stateUpdateSwapped(dsl::State & state);
	virtual std::string getArgumentForward() const;
	virtual std::string getArgumentBackward() const;
	virtual std::string getType();

private:
	enum Action { unselected, run, createRec, quit };

	rw::trajectory::TimedQPath trajectoryLoad();
	std::vector<rw::math::Q> convertTrajectory(rw::trajectory::TimedQPath trajectory);

	void updateTrajectory();
	Action userActionSelect();
	void createRecords(int numberToCreate);
	void moveTo(rw::math::Q q);



	std::shared_ptr<rwhw::URCallBackInterface> _ur;
	std::shared_ptr<rwhw::UniversalRobotsRTLogging> _urrt;
	std::shared_ptr<LearningProxy> _learningInterface;

	std::shared_ptr<ActionIKSolver> _ikSolver;

	std::shared_ptr<RecordDataset> _dataset;
	std::string _filepathDataset;

	std::string _filepathTrajectory;
	std::string _toolFrame;
	std::string _fixtureFrame;
	rw::math::Q _qInit;

	std::vector<rw::math::Q> _trajectory;
	rw::math::Q qStart;
	rw::math::Q qEnd;

	static const std::string _nameWorkcellFilePath;
	static const std::string _nameRobot;

	Action _action;

};

} /* namespace intelligentmove */
} /* namespace dsl */

#endif /* REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_INTELLIGENTACTIONMODELINTERFACE_HPP_ */


