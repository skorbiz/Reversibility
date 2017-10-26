/*
 * ManualExecutionSelector.cpp
 *
 *  Created on: Jun 30, 2016
 *      Author: josl
 */

#include "ManualExecutionSelector.hpp"

#include <assert.h>
#include "XMLRecordSaver.hpp"

namespace dsl {
namespace intelligentmove {

bool ManualExecutionSelector::staticCreateRecs = false;
bool ManualExecutionSelector::staticUseDefault = false;
bool ManualExecutionSelector::staticIgonoreED = false;
const int ManualExecutionSelector::nRecordRequriement = 15;

ManualExecutionSelector::ManualExecutionSelector(
		std::shared_ptr<IntelligentTrajectory> intTrajectortyForward,
		std::shared_ptr<IntelligentTrajectory> intTrajectortyBackward,
		std::shared_ptr<RecordDataset> set,
		std::string datasetFilepath)
			: _intTrajectoryForward(intTrajectortyForward)
			, _intTrajectoryBackward(intTrajectortyBackward)
			, _dataset(set)
			, _datasetFilepath(datasetFilepath)
			, _action(unselected)
{
}

ManualExecutionSelector::~ManualExecutionSelector()
{
}

void ManualExecutionSelector::execute()
{
	actionExecuter();
}

void ManualExecutionSelector::executeBackwards()
{
	_intTrajectoryForward.swap(_intTrajectoryBackward);
	actionExecuter();
	_intTrajectoryForward.swap(_intTrajectoryBackward);
}



void ManualExecutionSelector::actionExecuter()
{
	_action = actionSelecter();

	if(_action == createRecs)
	{
		createRecords(nRecordRequriement);
		_action = run;
	}

	if(_action == createRec)
	{
		if(getMatchingRecordsForward() == 0)
			createRecords(1);
		_action = runDefault;
	}


	if(_action == run)
		_intTrajectoryForward->executeWithErrorCatch();
	else if(_action == runDefault)
		_intTrajectoryForward->execute();
	else if(_action == runIgnoreED)
		_intTrajectoryForward->creatRecord();
	else
	{
		std::cerr << "quit in IMOVE" << std::endl;
		exit(1);
	}

}

ManualExecutionSelector::Action ManualExecutionSelector::actionSelecter()
{
	if( doDatasetContainsSufficinetRecrods())
		return run;

	//if( getMatchingRecordsForward() == 1)
	//	return runDefault;

	if(staticCreateRecs)
		return createRecs;
	if(staticUseDefault)
		return createRec;
	if(staticIgonoreED)
		return runIgnoreED;

	return actionSelecterUser();
}

ManualExecutionSelector::Action ManualExecutionSelector::actionSelecterUser()
{
	//Record has sufficient data -> execute

	std::cout << "IntelligentMove dataset did not contain enought records" << std::endl;
	std::cout << "IntelligentMove requries: " << nRecordRequriement << std::endl;
	std::cout << "dataset contained: " << std::endl;
	printRecordStatistics();

	std::string userInput;
	std::cout << "Select an action"
			"\n (q)  quit"
			"\n (r)  create records"
			"\n (ra) create records (for all)"
			"\n (d)  create single record and use default variance"
			"\n (da) create single record and use default variance (for all)"
			"\n (i)  ignore error detection (for all)"
			"\n (ia)  ignore error detection (for all)"
			"\n ";


//	std::cin  >> userInput;
	std::getline(std::cin, userInput);
	std::cout << "Registed input: " << userInput << std::endl;

	if(userInput == "r")
		return createRecs;

	if(userInput == "d")
		return createRec;

	if(userInput == "i")
		return runIgnoreED;

	if(userInput == "ra")
	{
		staticCreateRecs = true;
		return createRecs;
	}

	if(userInput == "da")
	{
		staticUseDefault = true;
		return createRec;
	}

	if(userInput == "ia")
	{
		staticIgonoreED = true;
		return runIgnoreED;
	}

	return quit;
}

bool ManualExecutionSelector::doDatasetContainsSufficinetRecrods()
{
	return doDatasetContainsSufficinetRecrodsForward() and doDatasetContainsSufficinetRecrodsBackward();
}

bool ManualExecutionSelector::doDatasetContainsSufficinetRecrodsForward()
{
	if(nRecordRequriement <= getMatchingRecordsForward())
		return true;
	return false;
}

bool ManualExecutionSelector::doDatasetContainsSufficinetRecrodsBackward()
{
	if(nRecordRequriement <= getMatchingRecordsBackward())
		return true;
	return false;
}

int ManualExecutionSelector::getMatchingRecordsForward()
{
	return _intTrajectoryForward->datasetSize();
}

int ManualExecutionSelector::getMatchingRecordsBackward()
{
	return _intTrajectoryBackward->datasetSize();
}


void ManualExecutionSelector::printRecordStatistics()
{
	std::cout << "# records in dataset: " << _dataset->getSize() << std::endl;
	std::cout << "# forward matching records: " << getMatchingRecordsForward() << std::endl;
	std::cout << "# backward matching records: " << getMatchingRecordsBackward() << std::endl;
}

void ManualExecutionSelector::createRecords(int numberToCreate)
{
	assert(!_datasetFilepath.empty());

	for(int i = 0; i < numberToCreate; i++)
	{
		std::cout << "learning " << i << std::endl;
		dsl::intelligentmove::Record rFor= _intTrajectoryForward->creatRecord();
		dsl::intelligentmove::Record rBck= _intTrajectoryBackward->creatRecord();

		_dataset->addRecord(rFor);
		_dataset->addRecord(rBck);
	}

	dsl::intelligentmove::XMLRecordSaver saver;
	saver.save(*_dataset, _datasetFilepath);
}


} /* namespace intelligentmove */
} /* namespace dsl */
