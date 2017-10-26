/*
 * ManualExecutionSelector.hpp
 *
 *  Created on: Jun 30, 2016
 *      Author: josl
 */

#ifndef REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_MANUALEXECUTIONSELECTOR_HPP_
#define REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_MANUALEXECUTIONSELECTOR_HPP_

#include "IntelligentTrajectory.hpp"
#include "RecordDataset.hpp"

namespace dsl {
namespace intelligentmove {

class ManualExecutionSelector {
public:
	ManualExecutionSelector(
			std::shared_ptr<IntelligentTrajectory> intTrajectortyForward,
			std::shared_ptr<IntelligentTrajectory> intTrajectortyBackward,
			std::shared_ptr<RecordDataset> set,
			std::string datasetFilepath);
	virtual ~ManualExecutionSelector();

	void execute();
	void executeBackwards();

private:
	enum Action { unselected, run, runDefault, runIgnoreED, createRec, createRecs, quit };

private:
	void printRecordStatistics();
	void createRecords(int numberToCreate);

	void actionExecuter();
	Action actionSelecter();
	Action actionSelecterUser();

	bool doDatasetContainsSufficinetRecrods();
	bool doDatasetContainsSufficinetRecrodsForward();
	bool doDatasetContainsSufficinetRecrodsBackward();

	int getMatchingRecordsForward();
	int getMatchingRecordsBackward();


private:
	std::shared_ptr<IntelligentTrajectory> _intTrajectoryForward;
	std::shared_ptr<IntelligentTrajectory> _intTrajectoryBackward;


	std::shared_ptr<RecordDataset> _dataset;
	std::string _datasetFilepath;

	Action _action;
	static bool staticCreateRecs;
	static bool staticUseDefault;
	static bool staticIgonoreED;

	static const int nRecordRequriement;

};

} /* namespace intelligentmove */
} /* namespace dsl */

#endif /* REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_MANUALEXECUTIONSELECTOR_HPP_ */
