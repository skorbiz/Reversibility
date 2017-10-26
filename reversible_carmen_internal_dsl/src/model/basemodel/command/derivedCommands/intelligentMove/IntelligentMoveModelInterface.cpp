/*
 * IntelligentMoveModelInterface.cpp
 *
 *  Created on: May 12, 2016
 *      Author: josl
 */

#include "IntelligentMoveModelInterface.hpp"
#include "ManualExecutionSelector.hpp"

namespace dsl {
namespace intelligentmove {


IntelligentMoveModelInterface::	IntelligentMoveModelInterface(std::shared_ptr<rwhw::URCallBackInterface> urc, std::shared_ptr<rwhw::UniversalRobotsRTLogging> urrt, rw::math::Q qTo, std::shared_ptr<RecordDataset> set, std::string datasetfilepath)
 : 	 _ur(urc)
	,_urrt(urrt)
	,_qEnd(qTo)
	,_dataset(set)
	,_datasetFilepath(datasetfilepath)
{
}

IntelligentMoveModelInterface::~IntelligentMoveModelInterface()
{
}

void IntelligentMoveModelInterface::execute()
{
	std::cout << "void IntelligentMoveModelInterface::execute()" << std::endl;

	std::shared_ptr<dsl::intelligentmove::IntelligentTrajectory> imFor;
	std::shared_ptr<dsl::intelligentmove::IntelligentTrajectory> imBck;

	imFor = std::make_shared<dsl::intelligentmove::IntelligentMove>(_ur,_urrt, _qStart, _qEnd, *_dataset);
	imBck = std::make_shared<dsl::intelligentmove::IntelligentMove>(_ur,_urrt, _qEnd, _qStart, *_dataset);

	dsl::intelligentmove::ManualExecutionSelector manualSelector(imFor, imBck, _dataset, _datasetFilepath);
	manualSelector.execute();
}

void IntelligentMoveModelInterface::executeBackwards()
{
	std::cerr << "void IntelligentMoveModelInterface::executeBackwards()" << std::endl;
	std::shared_ptr<dsl::intelligentmove::IntelligentTrajectory> imFor;
	std::shared_ptr<dsl::intelligentmove::IntelligentTrajectory> imBck;

	imFor = std::make_shared<dsl::intelligentmove::IntelligentMove>(_ur,_urrt, _qStart, _qEnd, *_dataset);
	imBck = std::make_shared<dsl::intelligentmove::IntelligentMove>(_ur,_urrt, _qEnd, _qStart, *_dataset);

	dsl::intelligentmove::ManualExecutionSelector manualSelector(imFor, imBck, _dataset, _datasetFilepath);
	manualSelector.executeBackwards();
}

void IntelligentMoveModelInterface::stateUpdate(dsl::State & state)
{
	_qStart = state.getQ();
	state.update(_qEnd);
}

void IntelligentMoveModelInterface::stateUpdateSwapped(dsl::State & state)
{
	state.update(_qStart);
}

std::string IntelligentMoveModelInterface::getType()
{
	return "IntelligentMove";
}

std::string IntelligentMoveModelInterface::getArgumentForward() const
{
	std::stringstream s;
	s << std::fixed;
	s << std::setprecision(2);
	s << _qEnd;
	return s.str();
}

std::string IntelligentMoveModelInterface::getArgumentBackward() const
{
	std::stringstream s;
	s << std::fixed;
	s << std::setprecision(2);
	s << _qStart;
	return s.str();
}



} /* namespace intelligentmove */
} /* namespace dsl */
