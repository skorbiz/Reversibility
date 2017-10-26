/*
 * TestIntDataExtraction2.cpp
 *
 *  Created on: Jul 5, 2016
 *      Author: josl
 */

#include "TestIntDataExtraction2.hpp"

#include <iostream>
#include <../src/model/basemodel/command/derivedCommands/Move.hpp>
#include <../src/model/basemodel/command/derivedCommands/IOManipulation.hpp>
#include <../src/model/basemodel/command/derivedCommands/intelligentMove/Record.hpp>
#include <../src/model/basemodel/command/derivedCommands/intelligentMove/RecordDataset.hpp>
#include <../src/model/basemodel/command/derivedCommands/intelligentMove/RecordPrintUtillities.hpp>
#include <../src/model/basemodel/command/derivedCommands/intelligentMove/IntelligentMove.hpp>
#include <../src/model/basemodel/command/derivedCommands/intelligentMove/Statistics.hpp>
#include <../src/common/common.hpp>
#include <../src/common/WorkcellModel.hpp>
#include "../model/basemodel/command/derivedCommands/intelligentMove/XMLRecordLoader.hpp"
#include "../model/basemodel/command/derivedCommands/intelligentMove/XMLRecordSaver.hpp"

#include "debug/Debug.hpp"

namespace dsl {
namespace intelligentmove {

TestIntDataExtraction2::TestIntDataExtraction2()
{


	std::cout << "Test of intelligent action" << std::endl;

	connectUR();
	connectIO();

	std::string filepath = dsl::common::getPath2dslOutputFolder() + "/iDataMachineSaves.xml";
	dsl::intelligentmove::XMLRecordSaver saver;
	dsl::intelligentmove::XMLRecordLoader loader;
	dsl::intelligentmove::RecordDataset dataset;


	dataset = loader.load(filepath);

	rw::math::Q pos_common (6, 2.365823, -1.64587, 1.880681, -1.78785, -1.55238, 2.382796 );

	rw::math::Q pos_machine_bend_before (6, 2.66612124, -1.5517557, 1.94304323, -1.9524143, -1.5480473, 2.68197417 );
	rw::math::Q pos_machine_bend_at (6, 2.74560999, -1.4740241, 1.87415647, -1.9635880, -1.5473530, 2.76173329 );
	rw::math::Q pos_machine_bend_in (6, 2.86369013, -1.4906099, 1.89352703, -1.9698708, -1.5467546, 2.87956881 );


	std::vector<rw::math::Q> trajectory;
	trajectory.push_back(pos_machine_bend_before);
	trajectory.push_back(pos_machine_bend_at);
	trajectory.push_back(pos_machine_bend_in);


	dsl::Move move1(_urrt,_ur, pos_common);
	dsl::Move move2(_urrt,_ur, pos_machine_bend_before);
	dsl::Move move3(_urrt,_ur, pos_machine_bend_at);
	dsl::Move move4(_urrt,_ur, pos_machine_bend_in);

	rw::math::Q qStart = trajectory.front();
	rw::math::Q qend = trajectory.back();

	int strategy = 5;
	IntelligentTrajectory itX(_ur,_urrt, trajectory, dataset.getRecordsWith(qStart,qend));
	itX.setUpdateStrategy(strategy);

	/*********************** CREATE NEW BASELINE MEASUREMENT ***********************************/
//	move1.execute();
//	move2.execute();
//	dsl::intelligentmove::Record r0 = itX.creatRecord();
//	move3.execute();
//	move2.execute();
//
//	dsl::intelligentmove::RecordDataset datasetNew;
//	datasetNew.addRecord(r0);
//	saver.save(datasetNew,filepath);
//	assert(false && "Stop program must be reloaded");

	///*********************** run with error catch ***********************************/


	int success = 0;
	int failed = 0;

	for(int i = 0; i < 50; i++)
	{
		move1.execute();
		move2.execute();
		bool result = itX.execute();

		if(result == true)
			success++;
		else
			failed++;

		std::string filename = "/exp_type_4_machine_success_number_" + std::to_string(i) + ".txt";
		std::string filepath = dsl::common::getPath2dropbox() + filename;
		dsl::intelligentmove::Record r = itX.getRecord();
		dsl::intelligentmove::RecordPrintUtillities p;
		p.printToFile(filepath, r, dataset);

		std::cout << dsl::color::BOLDMAGENTA;
		std::cout << "Experiment number: " << i << std::endl;
		std::cout << " Total: " << success + failed << std::endl;
		std::cout << " Success: " << success << std::endl;
		std::cout << " failed: " << failed << std::endl;
		std::cout << dsl::color::DEFAULT << std::endl;
		ros::Duration(1.0).sleep();

		move3.execute();
		move2.execute();

	}


}

TestIntDataExtraction2::~TestIntDataExtraction2()
{

}

void TestIntDataExtraction2::connectUR()
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

void TestIntDataExtraction2::connectIO()
{
	rw::common::Ptr<ros::NodeHandle> nodeHnd = rw::common::Ptr<ros::NodeHandle>(new ros::NodeHandle);
	_digital =  std::shared_ptr<DigitalIOGeneralInterfaceProxy>(new DigitalIOGeneralInterfaceProxy(nodeHnd, "Moxa")); //Moxa
}


} /* namespace intelligentmove */
} /* namespace dsl */
