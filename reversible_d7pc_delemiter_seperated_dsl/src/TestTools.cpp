/*
 * TestTools.cpp
 *
 *  Created on: Nov 20, 2016
 *      Author: josl
 */

// Bring in my package's API, which is what I'm testing
#include <math.h>
#include <ros/ros.h>
#include <ros/package.h>

#include "File.h"
#include "RegexIntepreter.h"

// Bring in gtest
#include <gtest/gtest.h>


TEST(MyTestCaseNameMultiply, Correct)
{
  EXPECT_EQ(1, 1*1);
  EXPECT_EQ(-4, -1*4);
  EXPECT_EQ(4, 2*2);
}

TEST(ToolsTest, File)
{
	std::string filepath = ros::package::getPath("reversible_assembly") + "/test/";
	std::string filename =  "test_input_file.txt";

	//ASSERT_DEATH(File("random file path", "filename"), "");

	File input(filepath, filename);
	EXPECT_FALSE(input.isEndOfFile());
	EXPECT_EQ("line 1", input.readLine());
	EXPECT_EQ("line 2", input.readLine());
	EXPECT_EQ("line 3", input.readLine());
	EXPECT_TRUE(input.isEndOfFile());
	EXPECT_EQ("", input.readLine());
}

TEST(ToolsTest, File2)
{
	std::string filepath = ros::package::getPath("reversible_assembly") + "/test/";
	std::string filename =  "test_input_file.txt";

	File input(filepath, filename);
	std::vector<std::string> lines = input.readLines();
	EXPECT_EQ(3, lines.size());
	EXPECT_EQ("line 1", lines[0]);
	EXPECT_EQ("line 2", lines[1]);
	EXPECT_EQ("line 3", lines[2]);
}


TEST(ToolsTest, RegexIntepreterCommand)
{
	EXPECT_STREQ("move", RegexIntepreter::extract_command("move : arg1 arg2").c_str());
	EXPECT_STREQ("move", RegexIntepreter::extract_command("move: arg1 arg2").c_str());
	EXPECT_STREQ("move", RegexIntepreter::extract_command("move:").c_str());
	EXPECT_STREQ("move", RegexIntepreter::extract_command("move").c_str());
	EXPECT_STREQ("move", RegexIntepreter::extract_command("move ").c_str());
	EXPECT_STREQ("move", RegexIntepreter::extract_command(" move ").c_str());

	EXPECT_STREQ("error", RegexIntepreter::extract_command("move : arg1 : arg2").substr(0,5).c_str());
}

TEST(ToolsTest, RegexIntepreterArgs)
{
	EXPECT_STREQ("arg1 arg2", RegexIntepreter::extract_args("move : arg1 arg2").c_str());
	EXPECT_STREQ("arg1 arg2", RegexIntepreter::extract_args("move: arg1 arg2").c_str());
	EXPECT_STREQ("", RegexIntepreter::extract_args("move:").c_str());
	EXPECT_STREQ("", RegexIntepreter::extract_args("move").c_str());
	EXPECT_STREQ("", RegexIntepreter::extract_args("move ").c_str());
	EXPECT_STREQ("", RegexIntepreter::extract_args(" move ").c_str());
}

TEST(ToolsTest, RegexIntepreterArgsList)
{
	EXPECT_EQ(2, RegexIntepreter::extract_args_list("move : arg1 arg2").size());
	EXPECT_STREQ("arg2", RegexIntepreter::extract_args_list("move: arg1 arg2")[1].c_str());
	EXPECT_STREQ("arg1", RegexIntepreter::extract_args_list("move : arg1 arg2")[0].c_str());
	EXPECT_STREQ("arg2", RegexIntepreter::extract_args_list("move : arg1 arg2")[1].c_str());
	EXPECT_STREQ("arg2", RegexIntepreter::extract_args_list("move : arg1 arg2 ")[1].c_str());
	EXPECT_STREQ("arg2", RegexIntepreter::extract_args_list("move : arg1  arg2")[1].c_str());
	EXPECT_EQ(0, RegexIntepreter::extract_args_list("move :").size());
	EXPECT_EQ(0, RegexIntepreter::extract_args_list("move  ").size());
}



//TEST(ToolsTest, Odometry)
//{
//	double wheel_radius = 1;
//	double wheel_base_dist = 2;
//	double encoder_pr_rev = 100;
//	epuck_driver_extended::Odometry odom(wheel_radius, wheel_base_dist, encoder_pr_rev);
//
//	odom.update(100, 100);
//	EXPECT_DOUBLE_EQ(2*M_PI*wheel_radius,odom.pos_x );
//	EXPECT_DOUBLE_EQ(0,odom.theta );
//
//	double threshold = 0.2;
//	for(int i = 0; i < 100; i++)
//		odom.update(100,100 + i);
//	EXPECT_NEAR(M_PI,odom.theta, threshold);
//	EXPECT_NEAR(2,odom.pos_y , threshold);
//
////	odom.update(20,20);
////	EXPECT_DOUBLE_EQ(0,odom.theta );
////	EXPECT_DOUBLE_EQ(2,odom.pos_y );
//}
//
//
//TEST(ToolsTest, Odometry_overflow)
//{
////	reversible_epuck_ant::Odometry odom1(-1, -1, -1);
////	int vlast = 0;
////	for(int i = 0; i < 10; i++ )
////	{
////		short val = 10000*i;
////		int vtrue = 10000*i;
////		vlast = odom1.overflow_detect(val, vlast, vtrue);
////	}
////
////
////	reversible_epuck_ant::Odometry odom2(-1, -1, -1);
////	int vlast = 0;
////	for(int i = 0; i < 10; i++ )
////	{
////		short val = -10000*i;
////		int vtrue = -10000*i;
////		vlast = odom2.overflow_detect(val, vlast, vtrue);
////	}
////
//
//	EXPECT_TRUE(true);
//
////	odom.update(20,20);
////	EXPECT_DOUBLE_EQ(0,odom.theta );
////	EXPECT_DOUBLE_EQ(2,odom.pos_y );
//}
//
//
//
//
//
//TEST(ToolsTest, ProximityAbstraction)
//{
//	epuck_driver_extended::ProximityAbstraction prox;
//
//	tf::Vector3 test (1,0,0);
//	tf::Vector3 res;
//
//	res = prox.worldTrobot(0,0,M_PI) * test;
//	EXPECT_NEAR(-1, res.getX(), 0.01);
//	EXPECT_NEAR(0, res.getY(), 0.01);
//
//	res = prox.worldTrobot(0,0,M_PI/2) * test;
//	EXPECT_NEAR(0, res.getX(), 0.01);
//	EXPECT_NEAR(1, res.getY(), 0.01);
//}
//
//
//
//
//
//
//
//TEST(ToolsTest, PositionFusion_full_odom)
//{
//	epuck_driver_extended::PositionFusion pf(1.0);
//	pf.update_from_odometry(0,0,0);
//	pf.update_from_vision(-1,4,2);
//	EXPECT_DOUBLE_EQ(0, pf.x());
//	EXPECT_DOUBLE_EQ(0, pf.y());
//	EXPECT_DOUBLE_EQ(0, pf.theta());
//
//	pf.update_from_odometry(1,2,3);
//	pf.update_from_vision(4,-2,-4);
//	EXPECT_DOUBLE_EQ(1, pf.x());
//	EXPECT_DOUBLE_EQ(2, pf.y());
//	EXPECT_DOUBLE_EQ(3, pf.theta());
//}
//
//
//TEST(ToolsTest, PositionFusion_full_vision)
//{
//	epuck_driver_extended::PositionFusion pf(0.0);
//	pf.update_from_odometry(0,0,0);
//	pf.update_from_vision(-1,4,2);
//	EXPECT_DOUBLE_EQ(-1, pf.x());
//	EXPECT_DOUBLE_EQ(4, pf.y());
//	EXPECT_DOUBLE_EQ(2, pf.theta());
//}
//
//
//TEST(ToolsTest, PositionFusion_90_odom)
//{
////	epuck_driver_extended::PositionFusion pf(0.90);
////	pf.update_from_odometry(1,1,1);
////	EXPECT_DOUBLE_EQ(0.9, pf.x());
////	EXPECT_DOUBLE_EQ(0.9, pf.y());
////	EXPECT_DOUBLE_EQ(0.9, pf.theta());
////
////	pf.update_from_vision(0,0,0);
////	EXPECT_DOUBLE_EQ(0.81, pf.x());
////	EXPECT_DOUBLE_EQ(0.81, pf.y());
////	EXPECT_DOUBLE_EQ(0.81, pf.theta());
//}
//
//
//TEST(ToolsTest, PositionFusion_consequtive_odom_updates)
//{
////	double k = 0.9;
////	epuck_driver_extended::PositionFusion pf(k);
////	pf.update_from_odometry(1,10,100);
////	EXPECT_DOUBLE_EQ(  1*k, pf.x());
////	EXPECT_DOUBLE_EQ( 10*k, pf.y());
////	EXPECT_DOUBLE_EQ(100*k, pf.theta());
////
////	pf.update_from_odometry(2,20,200);
////	EXPECT_DOUBLE_EQ(  1*k +   1*k*k, pf.x());
////	EXPECT_DOUBLE_EQ( 10*k +  10*k*k, pf.y());
////	EXPECT_DOUBLE_EQ(100*k + 100*k*k, pf.theta());
//}
//
//TEST(ToolsTest, RunningStatistics_1)
//{
//	int length = 102;
//	epuck_driver_extended::RunningStatistic rs(length,0);
//	EXPECT_EQ(0,rs.getFluctuationRatio());
//
//	for(int i = 0; i < length; i++)
//		if(i%2 == 0)	rs.add(1);
//		else			rs.add(-1);
//	EXPECT_NEAR(2, rs.getFluctuation(), 0.05);
//	EXPECT_NEAR(1, rs.getFluctuationRatio(), 0.01);
//	EXPECT_NEAR(length-2, rs.getFluctuationIndexs().size(), 0.01);
//
////	std::cout << "Ratio: ";
////	std::cout << rs.getFluctuationRatio() << std::endl;
////	std::cout << "Value: ";
////	std::cout << rs.getFluctuation() << std::endl;
////	std::cout << "count: ";
////	std::cout << rs.getFluctuationIndexs().size() << std::endl;
//
//}
//
//TEST(ToolsTest, RunningStatistics_2)
//{
//	int length = 102;
//	epuck_driver_extended::RunningStatistic rs(length,0);
//
//	for(int i = 0; i < length; i++)
//		if(i%4 == 0)	rs.add(1);
//		else			rs.add(-1);
//	EXPECT_NEAR(1, rs.getFluctuation(), 0.05);
//	EXPECT_NEAR(0.5, rs.getFluctuationRatio(), 0.01);
//	EXPECT_NEAR((length-2)/2, rs.getFluctuationIndexs().size(), 0.01);
//}
//
//TEST(ToolsTest, RunningStatistics_3)
//{
//	int length = 102;
//	epuck_driver_extended::RunningStatistic rs(length,0);
//
//	for(int i = 0; i < length; i++)
//		if(i%4 == 0)	rs.add(2);
//		else			rs.add(-2);
//	EXPECT_NEAR(2, rs.getFluctuation(), 0.05);
//	EXPECT_NEAR(0.5, rs.getFluctuationRatio(), 0.01);
//	EXPECT_NEAR((length-2)/2, rs.getFluctuationIndexs().size(), 0.01);
//}

