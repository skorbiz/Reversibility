/*
 * MapTest.cpp
 *
 *  Created on: Apr 30, 2015
 *      Author: josl
 */

#include "MapTest.hpp"

namespace dsl {

MapTest::MapTest()
{
}

MapTest::~MapTest()
{
}

void MapTest::buildExpandedModel()
{

//	// ******************************
//	// * RELATIVE MOVES TEST
//
//	JointConfiguration pos_screwdriver_take_infront(1.0019941733809739,
//			-0.9257137128081577, 1.4554540934745521, -0.5414931210429506,
//			2.5518927604385513, -3.1537604886637745);
//	JointConfiguration pos_common(0.7469845808274470, -1.6455190686395103,
//			1.9089228659138593, -1.84375670307725410, -1.57961682397111240,
//			-2.3945158228171133);
//
//	sequence("go_kvm_from_vola").move(pos_screwdriver_take_infront).move(
//			pos_common);
//
//	sequence("main").move(pos_common);
//	wait(1.0);
//	move(Vector3D<>(0, 0.1, 0), dsl::common::WorkcellModel::frame::tcp_kvm);
//	wait(1.0);
//	move(pos_common);
//
//
//
//	// ******************************
//	// * FORCE MODE PICKUP
//
//
//	rw::math::Q q0(6, 3.476714659271914, 4.034639226985933, 4.5898751502950486, -3.9121253536282694, 1.570796223792284, 4.377266971680633);
//	rw::math::Q q1(6, 0.00, -1.13, 1.64, -0.51, 4.05, -4.71 );
//	JointConfiguration q2 ( 2.00, -1.16, 1.85, -0.69, 3.97, -4.71 );
//	JointConfiguration q3 = {-2.00, -1.05, 1.73, -0.67, 4.00, -4.71 };
//	rw::math::Q q4(6, 2.9, 4.9, 4.02, -2.55, 2.98, 4.8);
//
//	std::string act_screw_basepath 		= ros::package::getPath("sdu_magic");
//	std::string act_screw_pathfile		= "/../../../scenes/TrajectoryOptimalVOlaPiH.txt";
//	std::string act_screw_path 			= act_screw_basepath + act_screw_pathfile;
//	std::string act_screw_tcp			= "Screwdriver_TCP";
//	std::string act_screw_fix2			=  "hole1_TCP";
//	std::string act_screw_fix1			=  "hole2_TCP";
//
//	sequence("move").
//		move(q4).
//		move(q1).
//		move(q2).
//		move(q3);
//		action(act_screw_basepath, act_screw_tcp, act_screw_fix1, q1);
//	move(q0);
//	move(Vector3D<>(0.1, 0, 0), dsl::common::WorkcellModel::frame::toolmount);
//	move(Vector3D<>(0, 0.1, 0), dsl::common::WorkcellModel::frame::toolmount);
//	move(Vector3D<>(0, 0, 0.1), dsl::common::WorkcellModel::frame::toolmount);
//
//	sequence("ABC").print("A").print("B");
//	print("C");
//	print("D");
//
//	sequence("QWTY").print("Q").print("W");
//	callReverse("ABC");
//	print("T");//	print("Y");
//



	JointConfiguration q1 (1,1,1,1,1,1 );
	JointConfiguration q2 (2,2,2,2,2,2 );
	JointConfiguration q3 (3,3,3,3,3,3 );

	JointConfiguration q4 (4,4,4,4,4,4 );
	JointConfiguration q5 (5,5,5,5,5,5 );
	JointConfiguration q6 (6,6,6,6,6,6 );

	JointConfiguration q7 (7,7,7,7,7,7 );
	JointConfiguration q8 (8,8,8,8,8,8 );
	JointConfiguration q9 (9,9,9,9,9,9 );

	sequence("move_123").
			move(q1).
			move(q2).
			move(q3);

	sequence("move_456").
			move(q4).
			move(q5).
			move(q6);

	sequence("move_789").
			move(q7).
			move(q8).
			move(q9);


	sequence("main").
			call("move_123").reverseWith("move_456");
			print("t").
			uncall("move_789");

}


} /* namespace dsl */
