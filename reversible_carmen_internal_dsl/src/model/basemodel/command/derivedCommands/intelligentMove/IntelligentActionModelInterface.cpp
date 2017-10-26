/*
 * IntelligentActionModelInterface.cpp
 *
 *  Created on: Jun 27, 2016
 *      Author: josl
 */

#include "IntelligentActionModelInterface.hpp"
#include <../src/model/basemodel/command/derivedCommands/ActionIKSolver.hpp>
#include "../src/common/common.hpp"
#include <../src/model/basemodel/command/derivedCommands/Move.hpp>

#include "XMLRecordSaver.hpp"

namespace dsl {
namespace intelligentmove {

const std::string IntelligentActionModelInterface::_nameWorkcellFilePath("/scenes/CARMENscene_3.0/Scene_DSL_apr15.wc.xml");
const std::string IntelligentActionModelInterface::_nameRobot("UR5");

IntelligentActionModelInterface::IntelligentActionModelInterface(
		std::shared_ptr<rwhw::URCallBackInterface> urc,
		std::shared_ptr<rwhw::UniversalRobotsRTLogging> urrt,
		std::shared_ptr<RecordDataset> set,
		std::string filepathDataset,
		std::string filepathTrajectory,
		std::string frameTool,
		std::string frameFixture,
		rw::math::Q qInit)
: 	_ur(urc),
	_urrt(urrt),
	_dataset(set),
	_filepathDataset(filepathDataset),
	_filepathTrajectory(filepathTrajectory),
	_toolFrame(frameTool),
	_fixtureFrame(frameFixture),
	_qInit(qInit),
	_action(unselected)
{
	assert(!_filepathTrajectory.empty() && "Filepath in IntelligentActionModelInterface was empty");
	assert(!_toolFrame.empty() && "Filepath in IntelligentActionModelInterface was empty");
	assert(!_fixtureFrame.empty() && "Filepath in IntelligentActionModelInterface was empty");

	std::string workcellFile = common::getPath2trunk() + _nameWorkcellFilePath;

	_ikSolver = std::make_shared<ActionIKSolver>(
			workcellFile,
			_nameRobot,
			_toolFrame,
			_fixtureFrame);

	rw::trajectory::TimedQPath tj1 = trajectoryLoad();
	_trajectory = convertTrajectory(tj1);

	rw::common::Ptr<ros::NodeHandle> nodeHnd = rw::common::Ptr<ros::NodeHandle>(new ros::NodeHandle);
	_learningInterface = std::make_shared<LearningProxy>(nodeHnd, "learning");
}

IntelligentActionModelInterface::~IntelligentActionModelInterface()
{
}


void IntelligentActionModelInterface::execute()
{
	std::cout << "void IntelligentActionModelInterface::execute()" << std::endl;

	assert(qStart.size() == 6);

	std::cout << "trajectory size: " << _trajectory.size() << std::endl;

	if(_trajectory.front() != qStart)
		_trajectory.insert(_trajectory.begin(), qStart);
	qEnd = _trajectory.back();

	std::cout << "trajectory size: " << _trajectory.size() << std::endl;

	std::cout << __PRETTY_FUNCTION__ << std::endl;
	std::cout << " qstart: " << qStart << std::endl;
	std::cout << " qEnd: " << qEnd << std::endl;

	dsl::intelligentmove::IntelligentTrajectory it(_ur,_urrt, _trajectory, _dataset->getRecordsWith(qStart,qEnd));

	if(_action == unselected)
		_action = userActionSelect();

	if(_action == createRec)
	{
		createRecords(1);
		_action = run;
	}

	it.setTrajectory(_trajectory);


	while(true)
	{
		dsl::Move m(_urrt, _ur, qStart);
		m.execute();


		updateTrajectory();
		it.setTrajectory(_trajectory);

		std::cout << "x4" << std::endl;
		bool successful = it.execute();
		std::cout << "x5" << std::endl;

		if(successful)
		{
			_learningInterface->sendResult(true);
			break;
		}

		_learningInterface->sendResult(false);

		std::cout << dsl::color::RED;
		std::cout << " --- Error ---";
		std::cout << dsl::color::DEFAULT;
		std::cout << std::endl;

		moveTo(_trajectory.front());

	}



}

void IntelligentActionModelInterface::executeBackwards()
{
	std::cerr << "void IntelligentActionModelInterface::executeBackwards()" << std::endl;
}

void IntelligentActionModelInterface::stateUpdate(dsl::State & state)
{
	qStart = state.getQ();
	assert(_trajectory.size() > 0);
	state.update(_trajectory.back());
}

void IntelligentActionModelInterface::stateUpdateSwapped(dsl::State & state)
{
	assert(_trajectory.size() > 0);
	state.update(_trajectory.front());
}

std::string IntelligentActionModelInterface::getType()
{
	return "IntelligentActionModelInterface";
}

std::string IntelligentActionModelInterface::getArgumentForward() const
{
	assert(_trajectory.size() > 0);
	std::stringstream s;
	s << std::fixed;
	s << std::setprecision(2);
	s << _trajectory.front();
	return s.str();
}

std::string IntelligentActionModelInterface::getArgumentBackward() const
{
	assert(_trajectory.size() > 0);
	std::stringstream s;
	s << std::fixed;
	s << std::setprecision(2);
	s << _trajectory.back();
	return s.str();
}


rw::trajectory::TimedQPath IntelligentActionModelInterface::trajectoryLoad()
{
	_ikSolver->setInitialQ(_qInit);
	rw::trajectory::TimedQPath path = _ikSolver->solve(_filepathTrajectory);
	assert(path.size() != 0);
	return path;
}

std::vector<rw::math::Q> IntelligentActionModelInterface::convertTrajectory(rw::trajectory::TimedQPath trajectory)
{
	std::vector<rw::math::Q> res;
	for(auto& elemt : trajectory)
		res.push_back(elemt.getValue());
	return res;
}

void IntelligentActionModelInterface::updateTrajectory()
{
	std::cout << std::setprecision(6);
	std::cout << "Waiting for transform" << std::endl;
	rw::math::Transform3D<double> offset;
	offset = _learningInterface->getTransform();
	std::cout << "Got transform: " << offset.P() << " " << rw::math::RPY<double>(offset.R());
	std::cout << std::endl;
	_ikSolver->setOffsetTransform(offset);
	rw::trajectory::TimedQPath tj1 = trajectoryLoad();
	_trajectory = convertTrajectory(tj1);
}


IntelligentActionModelInterface::Action IntelligentActionModelInterface::userActionSelect()
{
	std::cout << __PRETTY_FUNCTION__ << std::endl;
	std::cout << "qstart: " << qStart << std::endl;
	std::cout << "qEnd: " << qEnd << std::endl;

	dsl::intelligentmove::IntelligentTrajectory it(_ur,_urrt, _trajectory, _dataset->getRecordsWith(qStart,qEnd));

	std::cout << "Intelligent Action action selection" << std::endl;
	std::cout << "dataset contained: " << std::endl;
	std::cout << it.datasetSize() << std::endl;

	//if(it.datasetSize() > 0)
	//	return run;

	std::string userInput;
	std::cout << "Select an action"
			"\n (q)  quit"
			"\n (d)  create record"
			"\n (r)  run learning"
			"\n ";

	std::getline(std::cin, userInput);
	std::cout << "Registed input: " << userInput << std::endl;

	if(userInput == "r")
		return run;

	if(userInput == "d")
		return createRec;

	return quit;

}

void IntelligentActionModelInterface::moveTo(rw::math::Q q)
{
	_ur->moveL(q, 0.1, 0.5);

	while(true)
	{
		ros::Duration(1).sleep();
		double diff = (_urrt->getLastData().qActual - q).norm2();
		std::cout << std::setprecision(9) << diff << std::endl;
		if(diff < 0.001)
			break;
	}
	ros::Duration(1).sleep();

}


void IntelligentActionModelInterface::createRecords(int numberToCreate)
{
	std::vector<rw::math::Q> backwardsTrajector(_trajectory.begin(), _trajectory.end());
	std::reverse(backwardsTrajector.begin(), backwardsTrajector.end());

	dsl::intelligentmove::IntelligentTrajectory itFor(_ur,_urrt, _trajectory, _dataset->getRecordsWith(qStart,qEnd));
	dsl::intelligentmove::IntelligentTrajectory itBck(_ur,_urrt, backwardsTrajector, _dataset->getRecordsWith(qStart,qEnd));

	assert(!_filepathDataset.empty());

	for(int i = 0; i < numberToCreate; i++)
	{
		dsl::intelligentmove::Record rFor= itFor.creatRecord();
		dsl::intelligentmove::Record rBck= itBck.creatRecord();

		_dataset->addRecord(rFor);
		_dataset->addRecord(rBck);
	}

	dsl::intelligentmove::XMLRecordSaver saver;
	saver.save(*_dataset, _filepathDataset);
}




} /* namespace intelligentmove */
} /* namespace dsl */
