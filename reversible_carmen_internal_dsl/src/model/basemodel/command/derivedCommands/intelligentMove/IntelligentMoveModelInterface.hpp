/*
 * IntelligentMoveModelInterface.hpp
 *
 *  Created on: May 12, 2016
 *      Author: josl
 */

#ifndef REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_INTELLIGENTMOVEMODELINTERFACE_HPP_
#define REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_INTELLIGENTMOVEMODELINTERFACE_HPP_

#include "IntelligentMove.hpp"
#include <../src/model/basemodel/command/CommandExecutable.hpp>
//#include "XMLRecordSaver.hpp"

namespace dsl {
namespace intelligentmove {

class IntelligentMoveModelInterface : public dsl::CommandExecutable
{

public:
	IntelligentMoveModelInterface(std::shared_ptr<rwhw::URCallBackInterface> urc, std::shared_ptr<rwhw::UniversalRobotsRTLogging> urrt, rw::math::Q qTo, std::shared_ptr<RecordDataset> set, std::string datasetFilepath);
	virtual ~IntelligentMoveModelInterface();

	virtual void execute();
	virtual void executeBackwards();
	void stateUpdate(dsl::State & state);
	void stateUpdateSwapped(dsl::State & state);
	virtual std::string getArgumentForward() const;
	virtual std::string getArgumentBackward() const;
	virtual std::string getType();

private:
	std::shared_ptr<rwhw::URCallBackInterface> _ur;
	std::shared_ptr<rwhw::UniversalRobotsRTLogging> _urrt;
	rw::math::Q _qStart;
	rw::math::Q _qEnd;

	std::shared_ptr<RecordDataset> _dataset;
	std::string _datasetFilepath;

};

} /* namespace intelligentmove */
} /* namespace dsl */

#endif /* REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_INTELLIGENTMOVEMODELINTERFACE_HPP_ */
