/*
 * TestIntelligentMove.cpp
 *
 *  Created on: Mar 1, 2016
 *      Author: josl
 */

#include "TestIntelligentMove.hpp"

#include <rwhw/universalrobots/UniversalRobotsRTLogging.hpp>
#include <rwhw/universalrobots/URCallBackInterface.hpp>
#include <rwhw/universalrobots/URDefinitions.hpp>

#include <iostream>
#include <../src/model/basemodel/command/derivedCommands/intelligentMove/Record.hpp>
#include <../src/model/basemodel/command/derivedCommands/intelligentMove/RecordDataset.hpp>
#include <../src/model/basemodel/command/derivedCommands/intelligentMove/RecordPrintUtillities.hpp>
#include <../src/model/basemodel/command/derivedCommands/intelligentMove/IntelligentMove.hpp>
#include <../src/model/basemodel/command/derivedCommands/intelligentMove/Statistics.hpp>
#include <../src/common/common.hpp>
#include <../src/common/WorkcellModel.hpp>
#include "../model/basemodel/command/derivedCommands/intelligentMove/XMLRecordLoader.hpp"
#include "../model/basemodel/command/derivedCommands/intelligentMove/XMLRecordSaver.hpp"

namespace dsl {

TestIntelligentMove::TestIntelligentMove()
{
	std::cout << "Test of intelligent move" << std::endl;

//	std::string filepath = dsl::common::getPath2dslOutputFolder() + "/iMoveSaves.xml";
//	dsl::intelligentmove::XMLRecordSaver saver;
//	dsl::intelligentmove::XMLRecordLoader loader;
//	dsl::intelligentmove::RecordDataset dataset;
//
//	dataset = loader.load(filepath);
//
//	ros::NodeHandle nh;
//	std::shared_ptr<caros::SerialDeviceSIProxy> sip(new caros::SerialDeviceSIProxy(nh,"caros_universalrobot"));
//	std::shared_ptr<caros::UrProxy> urp(new caros::UrProxy(nh,"caros_universalrobot"));
//	dsl::common::WorkcellModel wc;
//
//	unsigned int urrt_port = 30003;
//	unsigned int ur_port = 30001;
//	std::string device_ip = "192.168.1.250";
//	std::string callback_ip = "192.168.1.240";
//	unsigned int numeric_callback_port = 33333;
//
//
////	rwhw::UniversalRobotsRTLogging urrt;
////	rwhw::URCallBackInterface ur;
//
//	std::shared_ptr<rwhw::UniversalRobotsRTLogging> urrt(new rwhw::UniversalRobotsRTLogging(universalrobots::CB31));
//	std::shared_ptr<rwhw::URCallBackInterface> ur(new rwhw::URCallBackInterface(universalrobots::CB31));
//
//	try
//	{
//		urrt->connect(device_ip, urrt_port);
//	}
//	catch (rw::common::Exception& exp)
//	{
//		std::cout << "Could not connect to urrt:" << exp.what() << std::endl;
//	}
//
//	try
//	{
//		ur->connect(device_ip, ur_port);
//	}
//	catch (rw::common::Exception& exp)
//	{
//		std::cout << "Could not connect to urrt:" << exp.what() << std::endl;
//	}
//
//	ur->startCommunication(callback_ip, numeric_callback_port);
//	urrt->start();
//
//	std::cout << "Connected to UR interface" << std::endl;
//
////	//Original test set
////	rw::math::Q q1(6, 3.15319, -1.4478, 1.37673, -1.4099, 1.57271, 2.60883);
////	rw::math::Q q2(6, 2.20541, -0.8386, 0.70459, -1.3793, 1.56800, 2.60880);
////	rw::math::Q q3(6, 1.40312, -1.4600, 1.68823, -1.7173, 1.57293, 2.60878);
////	rw::math::Q q4(6, 1.40169, -1.2605, 0.56003, -0.7888, 1.57509, 2.60431);
//
//	//Box test set
//	rw::math::Q q3(6, 1.3251073, -1.753568, 1.6261854, -1.442536, 4.7043156, 1.3272349);
//	rw::math::Q q4(6, 1.3265335, -1.550653, 2.2389111, -2.258156, 4.7054405, 1.3235337);
//	rw::math::Q q1(6, 2.3179018, -0.991876, 1.4687061, -2.045528, 4.7057037, 2.3164839);
//	rw::math::Q q2(6, 2.3184297, -1.076932, 0.8358640, -1.327704, 4.7055482, 2.3219072);
//
////	dsl::intelligentmove::IntelligentMove im1(ur,urrt, wc, q4, q1, dataset);
////	dsl::intelligentmove::IntelligentMove im2(ur,urrt, wc, q1, q2, dataset);
////	dsl::intelligentmove::IntelligentMove im3(ur,urrt, wc, q2, q3, dataset);
////	dsl::intelligentmove::IntelligentMove im4(ur,urrt, wc, q3, q4, dataset);
////
////	ur->moveJ(q4);
////	ros::Duration(1.5).sleep();
////	for(int i = 0; i < 5; i++)
////	{
////		im1.creatRecord();
////		im2.creatRecord();
////		im3.creatRecord();
////		im4.creatRecord();
////	}
////
////	dsl::intelligentmove::RecordDataset datasetNew;
////	for(int i = 0; i < 20; i++)
////	{
////	datasetNew.addRecord(im1.creatRecord());
////	datasetNew.addRecord(im2.creatRecord());
////	datasetNew.addRecord(im3.creatRecord());
////	datasetNew.addRecord(im4.creatRecord());
////	}
////	saver.save(datasetNew,filepath);
//
//
////	ur->moveJ(q4);
////	ros::Duration(1.5).sleep();
////	for(int i = 0; i < 10; i++)
////	{
////		dsl::intelligentmove::Record r1 = im1.creatRecord();
////		dsl::intelligentmove::Record r2 = im2.creatRecord();
////		dsl::intelligentmove::Record r3 = im3.creatRecord();
////		dsl::intelligentmove::Record r4 = im4.creatRecord();
////
////		if( i > 0 )
////		{
////			dsl::intelligentmove::RecordPrintUtillities::printToFile(dsl::common::getPath2dropbox() + "/testsp1-"+std::to_string(i)+".txt",r1,dataset);
////			dsl::intelligentmove::RecordPrintUtillities::printToFile(dsl::common::getPath2dropbox() + "/testsp2-"+std::to_string(i)+".txt",r2,dataset);
////			dsl::intelligentmove::RecordPrintUtillities::printToFile(dsl::common::getPath2dropbox() + "/testsp3-"+std::to_string(i)+".txt",r3,dataset);
////			dsl::intelligentmove::RecordPrintUtillities::printToFile(dsl::common::getPath2dropbox() + "/testsp4-"+std::to_string(i)+".txt",r4,dataset);
////		}
////	}
//
////	ur->moveJ(q4);
////	ros::Duration(1.5).sleep();
////	dsl::intelligentmove::Record r1 = im1.creatRecordWithCrash();
////	dsl::intelligentmove::RecordPrintUtillities::printToFile(dsl::common::getPath2dropbox() + "/test1crash-.txt",r1,dataset);
//
////	ur->moveJ(q4);
////	ros::Duration(1.5).sleep();
////	dsl::intelligentmove::Record r1 = im1.creatRecordWithStop();
////	dsl::intelligentmove::RecordPrintUtillities::printToFile(dsl::common::getPath2dropbox() + "/test1stop-.txt",r1,dataset);
//
////
////
////	ur->moveJ(q4);
////	ros::Duration(1.5).sleep();
////	while(true)
////	{
////		im1.executeWithErrorCatch();
////		im2.executeWithErrorCatch();
////		im3.executeWithErrorCatch();
////		im4.executeWithErrorCatch();
////	}
////
//
//
////	//Statistic test
////	std::vector<rw::math::Q> input;
////	for(int i = 0; i < 6; i++)
////	{
////		input.push_back(rw::math::Q(6,i));
////		rw::math::Q r = dsl::intelligentmove::Statistics::calcBackwardsAccumulation(input,3);
////		std::cout << r[0] << ",";
////	}
////	std::cout << " supposed to be: 0,1,3,6,9,12" << std::endl;
////	std::cout << dsl::intelligentmove::Statistics::calcBackwardsAccumulation(input,3,input.end()-1)[0];
////	std::cout << " supposed to be 9" << std::endl;
////
////	std::vector<rw::math::Q> ravg = dsl::intelligentmove::Statistics::calcRunningAvarge(input,3);
////	for(unsigned int i = 0; i < ravg.size(); i++)
////		std::cout << ravg[i][0] << ",";
////	std::cout << " supposed to be: 0,0.333,1,2,3,4" << std::endl;
////
////	std::vector<double> iravgwinput = {1,0,0,0,10,10,10,10};
////	std::vector<rw::math::Q> iravgw;
////	for(unsigned int i = 0; i < iravgwinput.size(); i++)
////		iravgw.push_back(rw::math::Q(6,iravgwinput[i]));
////	std::vector<rw::math::Q> ravgw = dsl::intelligentmove::Statistics::calcRunningAvarge(iravgw,4,true);
////	for(unsigned int i = 0; i < ravgw.size(); i++)
////		std::cout << ravgw[i][0] << ",";
////	std::cout << " supposed to be: 0.4,0.3.0.2,0.1,4.0,7.0,9.0,10" << std::endl;
////
//
//	dsl::intelligentmove::IntelligentMove im1(ur,urrt, q4, q1, dataset);
//	dsl::intelligentmove::IntelligentMove im2(ur,urrt, q1, q4, dataset);
//
//	dsl::intelligentmove::IntelligentMove im3(ur,urrt, q4, q3, dataset);
//	dsl::intelligentmove::IntelligentMove im4(ur,urrt, q3, q4, dataset);
//
//	ur->moveJ(q4);
//	ros::Duration(1.5).sleep();
//	for(int i = 0; i < 10; i++)
//	{
//		dsl::intelligentmove::Record r1 = im1.creatRecord();
//		dsl::intelligentmove::Record r2 = im2.creatRecord();
//		std::cout << i << std::endl;
//
//		if( i >= 0 )
//		{
//			dsl::intelligentmove::RecordPrintUtillities::printToFile(dsl::common::getPath2dropbox() + "/testForward14-"+std::to_string(i)+".txt",r1);
//			dsl::intelligentmove::RecordPrintUtillities::printToFile(dsl::common::getPath2dropbox() + "/testBackward14-"+std::to_string(i)+".txt",r2);
//		}
//	}
//
//	for(int i = 0; i < 10; i++)
//	{
//		dsl::intelligentmove::Record r1 = im3.creatRecord();
//		dsl::intelligentmove::Record r2 = im4.creatRecord();
//
//		if( i > 0 )
//		{
//			dsl::intelligentmove::RecordPrintUtillities::printToFile(dsl::common::getPath2dropbox() + "/testForward43-"+std::to_string(i)+".txt",r1);
//			dsl::intelligentmove::RecordPrintUtillities::printToFile(dsl::common::getPath2dropbox() + "/testBackward43-"+std::to_string(i)+".txt",r2);
//		}
//	}

}

TestIntelligentMove::~TestIntelligentMove()
{
}

} /* namespace dsl */
;
