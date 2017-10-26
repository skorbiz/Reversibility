/*
 * IntelligentTrajectory.cpp
 *
 *  Created on: Jun 27, 2016
 *      Author: josl
 */

#include "IntelligentTrajectory.hpp"
#include "../src/model/basemodel/command/derivedCommands/Move.hpp"

namespace dsl {
namespace intelligentmove {

IntelligentTrajectory::IntelligentTrajectory(std::shared_ptr<rwhw::URCallBackInterface> urc,
								 std::shared_ptr<rwhw::UniversalRobotsRTLogging> urrt,
								 std::vector<rw::math::Q> trajectory,
								 std::vector<Record> dataset) :
		_urrt(urrt),
		_ur(urc),
		_trajectory(trajectory),
		_forceStatistics(dataset),
		_forceStatisticsUpdateStrategy(-1),
		_nOffsetUpdates(0),
		_doErrorCalculation(true),
		_doStopOnError(true),
		_doRecCrashTest(false),
		_safetyModeCurrent(0.0),
		_timeStampRobotCurrent(0.0),
		_newData(false),
		_errorState(ErrorState::normal),
		_errorConfidense(0),
		_logedFreqPC(0)
{
//	MinimumTresholdCalculator mtc(dataset.getRecordsWith(qStart, qEnd));
//	mtc.print();
}



IntelligentTrajectory::~IntelligentTrajectory()
{
}

void IntelligentTrajectory::reset()
{
	_logedForceSamples.clear();
	_logedQSamples.clear();
	_logedTimeStamps.clear();
	_logedTimeStampsPC.clear();
	_logedTCPpositions.clear();
	_logedIActual.clear();
	_logedIControl.clear();
	_logedSafetyMode.clear();
	_logedErrorState.clear();

	_forceStatistics.reset();
	_nOffsetUpdates = 0;
	_errorState = ErrorState::normal;
	_errorConfidense = 0;
	_trajectoryIterator = _trajectory.begin();
}

bool IntelligentTrajectory::move()
{
	std::cout << __PRETTY_FUNCTION__ << std::endl;
	std::cout << "y1" << std::endl;
	DEB_FUNC();
	reset();

	while(true)
	{
		std::cout << "y2" << std::endl;
		ReadRobotData();

		if(!_newData)
			continue;

		std::cout << "y3" << std::endl;
		printInputData();

		std::cout << "y4" << std::endl;
		updateForceStatisticOffsets();

		std::cout << "y5" << std::endl;
		if(!stepErrorCalc())
			return false;

		std::cout << "y6" << std::endl;
		if(stepServoCalc())
			return true;

		std::cout << "y7" << std::endl;
		if(_logedForceSamples.size() > 1000 && _doRecCrashTest)
			return true;

	}
}


bool IntelligentTrajectory::stepErrorCalc()
{
	DEB_FUNC();
	//Error testing
	if(_doErrorCalculation)
	{
		_forceStatistics.update(_forceStatisticsUpdateStrategy, _qCurrent, _TCPcurrent, _iActualCurrent);

		calcErrorState();
		printError();

		if(_errorState == ErrorState::error && _doStopOnError)
		{
			_print << std::setprecision(6);
			_ur->servo(_qCurrent); //_ur->stopRobot(40000);
			_print << std::endl;
			ros::Duration(1).sleep();
			_ur->stopRobot(40000);
			return false;
		}

	}

	return true;
}


bool IntelligentTrajectory::stepServoCalc()
{
	DEB_FUNC();
	const double epsilonQ = 0.06;//0.02;
	rw::math::Q qTarget = *_trajectoryIterator;

	assert(qTarget.size() == 6);
	assert(_qCurrent.size() == 6);
	assert(_ur != nullptr);

	double diffToEnd =  (qTarget-_qCurrent).norm2();
	rw::math::Q deltaQ = (qTarget-_qCurrent)/diffToEnd*epsilonQ;
	rw::math::Q qAim = _qCurrent + deltaQ;

	// Servo to qTarget
	_ur->servo(qAim);	//_urp->servoQ(qTarget);

	// QTarget reached
	if(epsilonQ > diffToEnd)
		_trajectoryIterator++;

	// QTarget reached
	if(_trajectoryIterator == _trajectory.end())
	{
		_ur->servo(qTarget);//_urp->servoQ(_qEnd);
		ros::Duration(0.5).sleep();
		return true;
	}

	return false;
}




bool IntelligentTrajectory::execute()
{
	return move();
}


void IntelligentTrajectory::executeWithErrorCatch()
{

	while(true)
	{
		bool successful = move();

		if(successful)
			break;

		_print << dsl::color::RED;
		_print << " --- Error ---";
		_print << dsl::color::DEFAULT;
		_print << std::endl;

		dsl::Move m(_urrt, _ur, _trajectory.front());
		m.execute();
	}
}


void IntelligentTrajectory::setExpectedOffset(rw::math::Transform3D<double> offset)
{
	_expectedOffset = offset;
}

void IntelligentTrajectory::setUpdateStrategy(int strategyNumber)
{
	_forceStatisticsUpdateStrategy = strategyNumber;
}


Record IntelligentTrajectory::creatRecordWithCrash()
{
	_doRecCrashTest = true;
	_doStopOnError = false;
	move();
	_doRecCrashTest = false;
	_doStopOnError = true;
	return getRecord();
}

Record IntelligentTrajectory::creatRecordWithStop()
{
	move();
	while(_logedForceSamples.size() < 1000)
		ReadRobotData();
	return getRecord();
}

Record IntelligentTrajectory::creatRecord()
{
	_doErrorCalculation = false;
	_doStopOnError = false;
	move();
	_doErrorCalculation = true;
	_doStopOnError = true;
	return getRecord();
}


Record IntelligentTrajectory::getRecord()
{
	DEB_FUNC();
	Record rec;
	rec.acceleration = -1;
	rec.forceSample = _logedForceSamples;
	rec.isActive = true;
	rec.note = "";
	rec.qEnd = _trajectory.back();
	rec.qSample = _logedQSamples;
	rec.qStart = _trajectory.front();
	rec.speed = -1;
	rec.successfullMove = true;
	rec.type = Record::MoveType::servo;
	rec.tcpPoseSample = _logedTCPpositions;
	rec.timestamp = _logedTimeStamps;
	rec.timestampPC = _logedTimeStampsPC;
	rec.iActual = _logedIActual;
	rec.iControl = _logedIControl;
	rec.safetyMode = _logedSafetyMode;
	rec.errorState = _logedErrorState;
	return rec;
}

ForceStatisticsSingle IntelligentTrajectory::getForceStatistics()
{
	return _forceStatistics;
}

void IntelligentTrajectory::setTrajectory(std::vector<rw::math::Q> trajectory)
{
	_trajectory = trajectory;
}

void IntelligentTrajectory::datasetUpdate(std::vector<Record> dataset)
{
//	ForceStatistics fs(dataset.getRecordsWith(_trajectory.front(), _trajectory.back()));
	std::cerr << "IntelligentTrajectory use of outdated force statistics" << std::endl;
//	_forceStatistics = ForceStatistics(dataset.getRecordsWith(_trajectory.front(), _trajectory.back()));
}

unsigned int IntelligentTrajectory::datasetSize()
{
	std::cerr << "IntelligentTrajectory use of outdated force statistics" << std::endl;
	//	return _forceStatistics.sizeDataset();
	return 1;
}


//***

void IntelligentTrajectory::calcErrorState()
{
	DEB_FUNC();
	if(_logedForceSamples.size() < 10)
		return;

	double current = _iActualCurrent.norm2();
	double mean = _forceStatistics._meanCurrent.norm2();
	double std = _forceStatistics._stdDiviationCurrent.norm2();

	_errorState = ErrorState::normal;

//	if(current > mean + 3*std || current < mean -3*std)
//	{
//		_errorState = ErrorState::error;
//		_errorConfidense += std;
//	}
//	else
	if(current > mean + 2.5*std || current < mean - 2.5*std)
	{
		_errorState = ErrorState::aware;
		_errorConfidense += std::abs(current-mean)/std;
	}
	else
		_errorConfidense = 0;

	if(_errorConfidense > 6)
		_errorState = ErrorState::error;
}

void IntelligentTrajectory::updateForceStatisticOffsets()
{
	DEB_FUNC();

	if(!_doErrorCalculation)
		return;

	rw::math::Transform3D<double> unitTransform;

	if(_nOffsetUpdates == 0)
	{
		_forceStatistics.calcOffsets(_qCurrent, _TCPcurrent, unitTransform);
		_nOffsetUpdates++;
	}
	else if(_nOffsetUpdates == 1 && std::distance(_trajectory.begin(), _trajectoryIterator) == 2)
	{
		_forceStatistics.calcOffsets(_qCurrent, _TCPcurrent, _expectedOffset);
		_nOffsetUpdates++;
	}
}


void IntelligentTrajectory::ReadRobotData()
{
	DEB_FUNC();
	while(!_urrt->hasData())							//	rw::math::Q qC(7);
		std::cout << ".";								//	while(qC.size() != 6)
	rwhw::URRTData data = _urrt->getLastData();			//	 qC = _urp->getQActual();

	_qCurrent = data.qActual;							//	_qCurrent = qC;
	_forceCurrent = data.tcpForce;						//_forceCurrent = _urp->getTcpForce();
	_TCPcurrent = data.toolPoseActual;					//_TCPcurrent = _urp->getToolPose();
	_timeStampRobotCurrent = data.controllerTimeStamp;	//_timeStampRobotCurrent = _urp->getControllerTimeStamp();

	_iActualCurrent = data.iActual;
	_iControlCurret = data.iControl;
	_safetyModeCurrent = data.safetyMode;

	assert(_qCurrent.size() == 6);
	assert(_forceCurrent.size() == 6);
	assert(_TCPcurrent.size() == 6);

	updateFrequncyPC();

	logData();
}

void IntelligentTrajectory::logData()
{
	DEB_FUNC();
	_newData = false;

	if(_logedQSamples.size() != 0)
		if(_logedTimeStamps.back() == _timeStampRobotCurrent)
			return;

	_newData = true;
	_logedQSamples.push_back(_qCurrent);
	_logedForceSamples.push_back(_forceCurrent);
	_logedTimeStamps.push_back(_timeStampRobotCurrent);
	_logedTCPpositions.push_back(_TCPcurrent);
	_logedIActual.push_back(_iActualCurrent);
	_logedIControl.push_back(_iControlCurret);
	_logedSafetyMode.push_back(_safetyModeCurrent);
	_logedErrorState.push_back(_errorState);

	_logedTimeStampsPC.push_back(ros::Time::now().toSec());
}

double IntelligentTrajectory::calcFrequncyRobot() const
{
	DEB_FUNC();
	if(_logedTimeStamps.size() < 2)
		return -1;

	double t1 = _logedTimeStamps[_logedTimeStamps.size()-2];
	double t2 = _logedTimeStamps.back();
	return  1.0/(t2-t1);
}

void IntelligentTrajectory::updateFrequncyPC()
{
	DEB_FUNC();
	ros::Time t = ros::Time::now();
	ros::Duration d = t - _logedTimePC;
	_logedFreqPC = 1.0/d.toSec();
	_logedTimePC = t;
}

void IntelligentTrajectory::colorfunc(double v, double th1, double th2, double th3)
{
	if(std::abs(v) > th3)		_print << dsl::color::BOLDRED;
	else if(std::abs(v) > th2)	_print << dsl::color::RED ;
	else if(std::abs(v) > th1)	_print << dsl::color::YELLOW ;
	else						_print << dsl::color::DEFAULT ;
}

void IntelligentTrajectory::out(double v)
{
	_print << std::fixed;
	_print << std::setprecision(2);
	_print << std::setw(9);
	_print << v;
	_print << dsl::color::DEFAULT;
}

void IntelligentTrajectory::out1(double v)
{
	colorfunc(v, 2, 3, 4);
	out(v);
}

void IntelligentTrajectory::out2(double v)
{
	colorfunc(v, 0.5, 1, 1.5);
	out(v);
}

void IntelligentTrajectory::out3(double v)
{
	colorfunc(v, 0.2, 0.3, 0.4);
	out(v);
}

void IntelligentTrajectory::printInputData()
{
	DEB_FUNC();
	_print << std::endl;
	out(calcFrequncyRobot());
	out(_logedFreqPC);
	out(_logedQSamples.size());
}

void IntelligentTrajectory::printError()
{
	DEB_FUNC();
	if(_logedIActual.size() < 2)
		return;

	std::cout << std::fixed;
	std::cout << std::setprecision(2);


	_print << " |iAct, mean, std ";
	_print << _iActualCurrent.norm2() << " , ";
	_print << _forceStatistics._meanCurrent.norm2() << " , ";
	_print << _forceStatistics._stdDiviationCurrent.norm2() << " , ";

	_print << " |iAct - mean ";
	_print << std::abs(_iActualCurrent.norm2() -_forceStatistics._meanCurrent.norm2()) << " , ";


//	_print << " |Index %, index, sample size, dist-match ";
//	out(_forceStatistics._corrIndexPercantage);
//	_print << " , " << ((int) _forceStatistics._correspondingIndex);
//	_print << " , " << _logedQSamples.size();
//	out1(_forceStatistics._corrIndexBestMatch * 1000);

	_print << " |Err conf. ";
	out1(_errorConfidense);

//	_print << " |safety mode. ";
//	out1(_logedSafetyMode.back());

	_print << " |std ";
	out3(_forceStatistics._stdDiviationCurrent.norm2());

	_print << " |std div ";
	out2(std::abs(_iActualCurrent.norm2() -_forceStatistics._meanCurrent.norm2())/_forceStatistics._stdDiviationCurrent.norm2());


}
















} /* namespace intelligentmove */
} /* namespace dsl */
