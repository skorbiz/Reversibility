/*
 * IntelligentTrajectory.hpp
 *
 *  Created on: Jun 27, 2016
 *      Author: josl
 */

#ifndef REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_INTELLIGENTTRAJECTORY_HPP_
#define REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_INTELLIGENTTRAJECTORY_HPP_

#include <memory>
#include <tuple>
#include <math.h>
#include <assert.h>

#include <rw/math/Q.hpp>
#include <rw/trajectory/Trajectory.hpp>

#include <rwhw/universalrobots/URCallBackInterface.hpp>
#include <rwhw/universalrobots/UniversalRobotsRTLogging.hpp>
#include <rwhw/universalrobots/URDefinitions.hpp>

#include <rw/common/Ptr.hpp>

#include <../src/common/DataPrint.hpp>
#include <color/colors.hpp>
#include <debug/Debug.hpp>

#include <../src/model/basemodel/command/derivedCommands/intelligentMove/Record.hpp>
#include <../src/common/WorkcellModel.hpp>
#include "ForceStatistics.hpp"
#include "ForceStatisticsSingle.hpp"

using rw::common::Ptr;

namespace dsl {
namespace intelligentmove {

class IntelligentTrajectory {
public:

	enum ErrorState {normal = 0, aware = 1, error = 2};

public:
	IntelligentTrajectory(std::shared_ptr<rwhw::URCallBackInterface> urc, std::shared_ptr<rwhw::UniversalRobotsRTLogging> urrt, std::vector<rw::math::Q> trajectory, std::vector<Record> dataset);
	virtual ~IntelligentTrajectory();

	bool execute();
	void executeWithErrorCatch();

	Record getRecord();
	Record creatRecord();
	Record creatRecordWithCrash();
	Record creatRecordWithStop();
	ForceStatisticsSingle getForceStatistics();

	void setTrajectory(std::vector<rw::math::Q> trajectory);
	void setExpectedOffset(rw::math::Transform3D<double> offset);
	void setUpdateStrategy(int strategyNumber);
	void datasetUpdate(std::vector<Record> dataset);
	unsigned int datasetSize();


private:
	bool move();
	bool stepErrorCalc();
	bool stepServoCalc();


	void calcErrorState();
	void updateForceStatisticOffsets();

	double calcFrequncyRobot() const;

	void ReadRobotData();
	void updateFrequncyPC();
	void logData();
	void reset();

	void printInputData();
	void printError();

private:
	std::shared_ptr<rwhw::UniversalRobotsRTLogging> _urrt;
	std::shared_ptr<rwhw::URCallBackInterface> _ur;

	std::vector<rw::math::Q> _trajectory;
	std::vector<rw::math::Q>::iterator _trajectoryIterator;

	ForceStatisticsSingle _forceStatistics;
	rw::math::Transform3D<double> _expectedOffset;
	int _forceStatisticsUpdateStrategy;
	int _nOffsetUpdates;

	bool _doErrorCalculation;
	bool _doStopOnError;
	bool _doRecCrashTest;

	rw::math::Q _qCurrent;
	rw::math::Q _forceCurrent;
	rw::math::Q _TCPcurrent;
	rw::math::Q _iActualCurrent;
	rw::math::Q _iControlCurret;
	double _safetyModeCurrent;
	double _timeStampRobotCurrent;
	bool _newData;

	ErrorState _errorState;
	double _errorConfidense;

	double _logedFreqPC;
	ros::Time _logedTimePC;
	std::vector<double> _logedTimeStamps;
	std::vector<double> _logedTimeStampsPC;
	std::vector<rw::math::Q> _logedQSamples;
	std::vector<rw::math::Q> _logedForceSamples;
	std::vector<rw::math::Q> _logedTCPpositions;
	std::vector<rw::math::Q> _logedIActual;
	std::vector<rw::math::Q> _logedIControl;
	std::vector<double> _logedSafetyMode;
	std::vector<double> _logedErrorState;

	debug::Debug _deb;
	debug::Debug _print;
	void colorfunc(double v, double th1, double th2, double th3);
	void out(double v);
	void out1(double v);
	void out2(double v);
	void out3(double v);


};

} /* namespace intelligentmove */
} /* namespace dsl */

#endif /* REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_COMMAND_DERIVEDCOMMANDS_INTELLIGENTMOVE_INTELLIGENTTRAJECTORY_HPP_ */
