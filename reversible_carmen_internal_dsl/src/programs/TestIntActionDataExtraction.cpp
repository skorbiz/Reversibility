/*
 * TestIntActionDataExtraction.cpp
 *
 *  Created on: Jul 5, 2016
 *      Author: josl
 */

#include "TestIntActionDataExtraction.hpp"

#include <iostream>
#include <../src/model/basemodel/command/derivedCommands/Move.hpp>
#include <../src/model/basemodel/command/derivedCommands/intelligentMove/Record.hpp>
#include <../src/model/basemodel/command/derivedCommands/intelligentMove/RecordDataset.hpp>
#include <../src/model/basemodel/command/derivedCommands/intelligentMove/RecordPrintUtillities.hpp>
#include <../src/model/basemodel/command/derivedCommands/intelligentMove/IntelligentMove.hpp>
#include <../src/model/basemodel/command/derivedCommands/intelligentMove/Statistics.hpp>
#include <../src/common/common.hpp>
#include <../src/common/WorkcellModel.hpp>
#include "../model/basemodel/command/derivedCommands/intelligentMove/XMLRecordLoader.hpp"
#include "../model/basemodel/command/derivedCommands/intelligentMove/XMLRecordSaver.hpp"

#include <dsl_common/MyCkTest.h>
#include "debug/Debug.hpp"

namespace dsl {
namespace intelligentmove {

TestIntActionDataExtraction::TestIntActionDataExtraction()
{


	std::cout << "Test of intelligent action" << std::endl;

	connectUR();
	connectIKSolver();
	connectLearningInterface();

	std::string filepath = dsl::common::getPath2dslOutputFolder() + "/iActionSaves.xml";
	dsl::intelligentmove::XMLRecordSaver saver;
	dsl::intelligentmove::XMLRecordLoader loader;
	dsl::intelligentmove::RecordDataset dataset;


	dataset = loader.load(filepath);

	rw::math::Q pos_common (6, 2.365823, -1.64587, 1.880681, -1.78785, -1.55238, 2.382796 );
	rw::math::Q outside_far 	(6, 2.563623, -1.55670, 2.234624, -2.23731, -1.57708, 2.555370);
	rw::math::Q above_inside 	(6, 2.680894, -1.64686, 2.216617, -2.12918, -1.57614, 2.673222);
	rw::math::Q above_tilted 	(6, 2.517352, -1.36697, 1.685097, -1.31548, -0.92325, 2.340487);

	rw::math::Q pos_drop_at (6, 2.0913, -1.278, 1.4849, -1.822, -1.600, 2.1096 );
	rw::math::Q pos_drop_up (6, 2.3153, -1.369, 1.9475, -3.166, -2.232, 1.9490 );


	rw::trajectory::TimedQPath tj = trajectoryLoad();
	std::vector<rw::math::Q> trajectory = convertTrajectory(tj, outside_far);


	dsl::Move move1(_urrt,_ur, pos_common);
	dsl::Move move2(_urrt,_ur, outside_far);
	dsl::Move move4(_urrt,_ur, above_inside);
	dsl::Move move5(_urrt,_ur, above_tilted);

	dsl::Move move6Drop(_urrt,_ur, pos_drop_at);
	dsl::Move move7Drop(_urrt,_ur, pos_drop_up);


	rw::math::Q qStart = trajectory.front();
	rw::math::Q qend = trajectory.back();



//	/*********************** CREATE NEW BASELINE MEASUREMENT ***********************************/
//	IntelligentTrajectory it(_ur,_urrt, trajectory, dataset.getRecordsWith(qStart,qend));
//	dsl::intelligentmove::RecordDataset datasetNew;
//	move1.execute();
//	move2.execute();
//	dsl::intelligentmove::Record r1 = it.creatRecord();
//	move4.execute();
//
//	datasetNew.addRecord(r1);
//	saver.save(datasetNew,filepath);
//	assert(false && "Stop program must be reloaded");


	///*********************** CREATE RECORD PRINT ***********************************/
	//	IntelligentTrajectory it3(_ur,_urrt, trajectory, dataset.getRecordsWith(qStart,qend));
	//	for(int i = 0; i < 12; i++)
	//	{
	//		move1.execute();
	//		move2.execute();
	//		dsl::intelligentmove::Record r1 = it3.creatRecord();
	//		move4.execute();
	//
	//		dsl::intelligentmove::RecordPrintUtillities::printToFile(dsl::common::getPath2dropbox() + "/testAct-"+std::to_string(i)+".txt",r1);
	//	}



	/*********************** EXPERIMENT WITH LEARNING INTERFACE ***********************************/

		assert(_learningInterface != nullptr);

		std::string filepathResult = dsl::common::getPath2dslOutputFolder() + "/resultsExpBF.txt";

		std::ofstream myfile;
		myfile.open (filepathResult);

		if (!myfile.is_open())
		{
			std::cerr << "Unable to open file to save imove" << std::endl;
			exit(-1);
		}

		myfile << "i ";
		myfile << "offsetX ";
		myfile <<"offsetY ";
		myfile << "succeded ";
		myfile << "completionP ";
		myfile << "completionSampleN ";
		myfile << "completionFinalMatch ";
		myfile << "\n";
		myfile.flush();

		int strategy = 5;
		int i = 0;

		while(true)
		{
			std::cout << "Fetching getTransform" << std::endl;
			rw::math::Transform3D<double> offset = _learningInterface->getTransform();
			std::cout << "Offset: " << offset.P()*1000 << std::endl;
				trajectory = getOffsetTrajectory(offset, outside_far);

			std::cout << "Creating imove" << std::endl;
			IntelligentTrajectory itX(_ur,_urrt, trajectory, dataset.getRecordsWith(qStart,qend));
			itX.setExpectedOffset(offset);
			itX.setUpdateStrategy(strategy);

			std::cout << dsl::color::BLUE;
			std::cout << "-- Experiment -- " << std::endl;
			std::cout << "i: " << i << std::endl;
			std::cout << "Offset: " << offset.P()*1000 << std::endl;
			std::cout << dsl::color::DEFAULT << std::endl;

			move1.execute();
			move2.execute();
			bool r = itX.execute();
			if(r == false)
			{
				std::cout << dsl::color::BOLDRED << "### FAILED ###" << dsl::color::DEFAULT << std::endl;
				_learningInterface->sendResult(false);
				move2.execute();
			}
			else
			{
				_learningInterface->sendResult(true);
				move4.execute();
				move6Drop.execute();
				move7Drop.execute();
			}

			myfile << i << " ";
			myfile << offset.P()[0] << " ";
			myfile << offset.P()[1] << " ";
			myfile << r << " ";
			myfile << itX.getForceStatistics()._corrIndexPercantage << " ";
			myfile << itX.getForceStatistics()._corrIndexDirect << " ";
			myfile << itX.getForceStatistics()._correspondingIndex << " \n";
			myfile.flush();

			std::cout << dsl::color::GREEN << std::endl;
			std::cout << std::setprecision(8);
			std::cout << "-- Result --" << std::endl;
			std::cout << "i: " << i << std::endl;
			std::cout << "offset x: " << offset.P()[0] << std::endl;
			std::cout << "offset y: " << offset.P()[1] << std::endl;
			std::cout << "result: " << r << std::endl;
			std::cout << "comp %: " << itX.getForceStatistics()._corrIndexPercantage << std::endl;
			std::cout << "comp n: " << itX.getForceStatistics()._corrIndexDirect << std::endl;
			std::cout << "comp i: " << itX.getForceStatistics()._correspondingIndex << std::endl;
			std::cout << dsl::color::DEFAULT << std::endl;
			i++;
		}
		myfile.close();


}

TestIntActionDataExtraction::~TestIntActionDataExtraction()
{

}

void TestIntActionDataExtraction::connectUR()
{
		unsigned int urrt_port = 30003;
		unsigned int ur_port = 30001;
		std::string device_ip = "192.168.1.250";	
		std::string callback_ip = "192.168.1.240";
		unsigned int numeric_callback_port = 33333;

		_urrt = std::make_shared<rwhw::UniversalRobotsRTLogging>(universalrobots::CB31);
		_ur = std::make_shared<rwhw::URCallBackInterface>(universalrobots::CB31);

		try
		{
			_urrt->connect(device_ip, urrt_port);
		}
		catch (rw::common::Exception& exp)
		{
			std::cout << "Could not connect to urrt:" << exp.what() << std::endl;
		}

		try
		{
			_ur->connect(device_ip, ur_port);
		}
		catch (rw::common::Exception& exp)
		{
			std::cout << "Could not connect to urrt:" << exp.what() << std::endl;
		}

		_ur->startCommunication(callback_ip, numeric_callback_port);
		_urrt->start();

		std::cout << "Connected to UR interface" << std::endl;
}



void TestIntActionDataExtraction::connectIKSolver()
{
	std::string filepathworkcell 		= common::getPath2trunk() + "/scenes/CARMENscene_3.0/Scene_DSL_apr15.wc.xml";
	std::string frameRobot				= "UR5";
	std::string frameTool				= "TCP_KVM";
	std::string frameFixture			= "HoleACP1";
	std::string frameFixture2			= "HoleACP2";

	_ikSolver = std::make_shared<ActionIKSolver>(
			filepathworkcell,
			frameRobot,
			frameTool,
			frameFixture);

	std::cout << "connected to IKSolver" << std::endl;
}


void TestIntActionDataExtraction::connectLearningInterface()
{
	rw::common::Ptr<ros::NodeHandle> nodeHnd = rw::common::Ptr<ros::NodeHandle>(new ros::NodeHandle);
	_learningInterface = std::make_shared<LearningProxy>(nodeHnd, "learning");
}

rw::trajectory::TimedQPath TestIntActionDataExtraction::trajectoryLoad()
{
	std::string filepathTrajectory = common::getPath2trunk() + "/scenes/KVM_pih_legacy/Trajectories/FemaleTmale_TCP_Good1453.txt";
	rw::math::Q qInit(6, 2.563623, -1.55670, 2.234624, -2.23731, -1.57708, 2.555370);

	_ikSolver->setInitialQ(qInit);
	rw::trajectory::TimedQPath path = _ikSolver->solve(filepathTrajectory);
	assert(path.size() != 0);
	return path;
}

std::vector<rw::math::Q> TestIntActionDataExtraction::convertTrajectory(rw::trajectory::TimedQPath trajectory, rw::math::Q qStart)
{
	std::vector<rw::math::Q> res;
	res.push_back(qStart);
	for(auto& elemt : trajectory)
		res.push_back(elemt.getValue());
	return res;
}

std::vector<rw::math::Q> TestIntActionDataExtraction::getOffsetTrajectory(rw::math::Transform3D<double> offset, rw::math::Q qStart)
{
	_ikSolver->setOffsetTransform(offset);
	rw::trajectory::TimedQPath tj1 = trajectoryLoad();
	return convertTrajectory(tj1, qStart);
}



} /* namespace intelligentmove */
} /* namespace dsl */
