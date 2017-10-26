/*
 * MapKVMIntMove.cpp
 *
 *  Created on: May 12, 2016
 *      Author: josl
 */

#include "MapKVMIntMove.hpp"


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

MapKVMIntMove::MapKVMIntMove()
{
}

MapKVMIntMove::~MapKVMIntMove()
{
}

void MapKVMIntMove::buildExpandedModel()
{

	// ******************************
	// * IO DEFS

	IOPorts gripper_tube(0);
	IOPorts gripper_nut(1);
	IOPorts gripper_rotater(2);
	IOPorts rotater(3);
	IOPorts machine(4);

	macro("tube_grasp").			io(gripper_tube, Switch::on);
	macro("tube_release").			io(gripper_tube, Switch::off);

	macro("nut_grasp").				io(gripper_nut, Switch::on);
	macro("nut_release").			io(gripper_nut, Switch::off);

	macro("rotater_position_1").	io(rotater, Switch::on);
	macro("rotater_position_2").	io(rotater, Switch::off);

	macro("rotater_grasp").			io(gripper_rotater, Switch::on);
	macro("rotater_release").		io(gripper_rotater, Switch::off);

	macro("machine_close").			io(machine, Switch::on);
	macro("machine_open").			io(machine, Switch::off);




	// ******************************
	// * VOLA TO KVM

	JointConfiguration pos_screwdriver_take_infront(0.849006658158743, -0.9276855492371849, 1.4052383056030697, -0.4775813335093756, -3.8633762620935084, -1.5707727416277333);
	JointConfiguration pos_common( 2.365823, -1.64587, 1.880681, -1.78785, -1.55238, 2.382796 );

	sequence("go_kvm_from_vola").
			move(pos_screwdriver_take_infront).
			move(pos_common);



	// ******************************
	// * ALTERNATIVE DROP

//	JointConfiguration pos_drop_at ( 2.251494, -1.33740, 1.864590, -3.00273, -2.17083, 1.987938 );
//	JointConfiguration pos_drop_up ( 2.251494, -1.33740, 1.864590, -3.00273, -2.17083, 1.987938 );

	JointConfiguration pos_drop_at ( 2.0913, -1.278, 1.4849, -1.822, -1.600, 2.1096 );
	JointConfiguration pos_drop_up ( 2.3153, -1.369, 1.9475, -3.166, -2.232, 1.9490 );
	// ******************************
	// * PICK  TUBE

//	JointConfiguration pos_tube_take_at 	( 2.671409, -1.24323, 1.786259, -2.10928, -1.55744, 2.686870 );
//	JointConfiguration pos_tube_take_in 	( 2.759592, -1.26937, 1.823215, -2.12192, -1.55704, 2.774831 );
//	JointConfiguration pos_tube_take_above 	( 2.759508, -1.32099, 1.761716, -2.00871, -1.55737, 2.775298 );

	JointConfiguration pos_tube_take_at 	( 2.629524, -1.21711, 1.739868, -2.08246, -1.57691, 2.622365 );
	JointConfiguration pos_tube_take_in 	( 2.721663, -1.25121, 1.788103, -2.09638, -1.57587, 2.714562 );
	JointConfiguration pos_tube_take_above 	( 2.718845, -1.30165, 1.722485, -1.98044, -1.57615, 2.712467 );

	sequence("drop_tube").
			move(pos_drop_at).
			call("tube_release").	neverReversible().
			wait(1.0).				neverReversible().
			move(pos_drop_up).		neverReversible();

	sequence("pick_tube").reverseWith("drop_tube").
		call("tube_release").
		wait(0.5).
		move(pos_tube_take_at).
		move(pos_tube_take_in).
		uncall("tube_release").
		wait(0.5).
		move(pos_tube_take_above).	neverReversible();


//	// ******************************
//	// * ROTATE TUBE
//
//	JointConfiguration pos_tube_rotate_at	(  );
//	JointConfiguration pos_tube_rotate_in	(  );
//	JointConfiguration pos_tube_rotate_out	(  );
//
//	sequence("rotate_tube").
//		call("rotater_release").
//		call("rotater_position_1").
//		wait(0.5).
//		move(pos_tube_rotate_at).
//		move(pos_tube_rotate_in).
//		uncall("rotater_release").
//		wait(0.5).
//		call("tube_release").
//		wait(0.5).
//		move(pos_tube_rotate_out).
//		call("rotater_position_2").
//		wait(0.5).
//		move(pos_tube_rotate_in).
//		uncall("tube_release").
//		wait(0.5).
//		call("rotater_release").
//		move(pos_tube_rotate_at).
//		uncall("rotater_release");
//
//
//	// ******************************
//	// * BEND TUBE
//
//	JointConfiguration pos_machine_bend_before (  );
//	JointConfiguration pos_machine_bend_at (  );
//	JointConfiguration pos_machine_bend_in (  );
//
//	sequence("bend_tube").
//		call("machine_open").
//		wait(0.5).
//		move(pos_machine_bend_before).
//		move(pos_machine_bend_at).
//		move(pos_machine_bend_in).
//		uncall("machine_open").
//		wait(2.0).
//		call("machine_open").
//		wait(0.5).
//		move(pos_machine_bend_in).
//		move(pos_machine_bend_at).
//		move(pos_machine_bend_before).
//		uncall("machine_open");
//

//	// ******************************
//	// * NUT GRASP AND DROP
//
//	JointConfiguration nutGrasp_at 			();
//	JointConfiguration nutGrasp_angle_up 	();
//	JointConfiguration nutGrasp_angle_down 	();
//
//
//	sequence("fixat_nut").
//		move(nutGrasp_at).
//		move(nutGrasp_angle_up).
//		call("nut_grasp");
//		move(nutGrasp_at);
//
//	sequence("drop_nut").
//		move(nutGrasp_at).
//		move(nutGrasp_angle_down).		neverReversible().
//		wait(1.0).						neverReversible().
//		move(nutGrasp_angle_down).		neverReversible();
//
//
//	// ******************************
//	// * FORCE MODE PICKUP
////
////	JointConfiguration outside_farUP 	(0.9773977629684002, -1.6960783148093999, 2.044482914887139, -1.9296525445258057, -1.5635200924465078, -3.734964237615029);
////	JointConfiguration outside_far 		(1.0290378711761934, -1.5245435657108464, 2.298115555152767, -2.355282538571036, -1.5640300451847038, -3.68342017018434);
////	JointConfiguration outside_close 	(1.0546417426459707, -1.5315042400245804, 2.305532818848015, -2.3559630627823176, -1.5642807431855097, -3.657778159317112);
////	JointConfiguration inside_deep 		(1.1067397505105343, -1.5431533732184066, 2.3178999573159573, -2.3570077036989145, -1.5648964567085004, -3.6057565564765977);
////	JointConfiguration inside_backplate (1.1269751174590166, -1.5451346372376928, 2.3215284546463812, -2.359250479484704, -1.569224188022293, -3.5870126534809788);
////	JointConfiguration above_inside 	(1.1067397505105343, -1.6403470478823836, 2.249989848062712, -2.1919148310132366, -1.564880254036428, -3.605724151132452);
////
////	dsl::ForceModeArgument deactivate = dsl::ForceModeArgument::getDeativationArgument();
////	dsl::ForceModeArgument forward = dsl::ForceModeArgument::getPushForward(30);
////	dsl::ForceModeArgument backward = dsl::ForceModeArgument::getPushForward(-30);
////
////	dsl::PositionJointComparison * inside_nut = new dsl::PositionJointComparison(inside_backplate, 0.03, _pRobotGeneral);
////
////	sequence("pick_nut").reverseWith("drop_nut").
////		move(outside_farUP).
////		move(outside_far).
////		move(outside_close).
////		forceMode(forward,deactivate).
////		wait(2).
////		forceMode(deactivate,deactivate).
////		check(inside_nut);
////		move(inside_deep).
////		move(above_inside);
////
////	sequence("pick_nuts").
////		call("pick_nut").
////		call("fixat_nut");
////		call("pick_nut");
//
//
	//******************************
	//* PIH ASSEMBLY 1

//	JointConfiguration outside_farUP 	();
//	JointConfiguration outside_far 		(2.606838, -1.58023, 2.273406, -2.25818, -1.55710, 2.621036);
	JointConfiguration outside_far 		(2.563623, -1.55670, 2.234624, -2.23731, -1.57708, 2.555370);
//	JointConfiguration outside_close 	();
//	JointConfiguration inside_deep 		();
//	JointConfiguration inside_backplate ();
//	JointConfiguration above_inside 	(2.728942, -1.65040, 2.264187, -2.18095, -1.55683, 2.743679);
	JointConfiguration above_inside 	(2.680894, -1.64686, 2.216617, -2.12918, -1.57614, 2.673222);


	std::string act_nut_pathfile1		= "/../../../scenes/KVM_pih_legacy/Trajectories/FemaleTmale_TCP_Good1453.txt"; //8
//	std::string act_nut_pathfile2		= "/../../../scenes/KVM_pih_legacy/Trajectories/FemaleTmale_TCP_Bad164.txt";  //8
//	std::string act_nut_pathfile3		= "/../../../scenes/KVM_pih_legacy/Trajectories/FemaleTmale_TCP_Bad702.txt";  //5
//	std::string act_nut_pathfile4		= "/../../../scenes/KVM_pih_legacy/Trajectories/FemaleTmale_TCP_Bad1221.txt"; //8
	std::string act_nut_basepath 		= ros::package::getPath("sdu_magic");
	std::string act_nut_path1 			= act_nut_basepath + act_nut_pathfile1;
//	std::string act_nut_path2 			= act_nut_basepath + act_nut_pathfile2;
//	std::string act_nut_path3 			= act_nut_basepath + act_nut_pathfile3;
//	std::string act_nut_path4 			= act_nut_basepath + act_nut_pathfile4;
	std::string act_nut_tcp				= "TCP_KVM";
	std::string act_nut_fix1			= "HoleACP1";
	std::string act_nut_fix2			= "HoleACP2";
//	std::string act_nut_fix1			= "HoleACP_test";
//	std::string act_nut_fix2			= "HoleACP_test";

	sequence("action_pih1").
		move(outside_far).
		action(act_nut_path1, act_nut_tcp, act_nut_fix1, pos_common).
//		action(act_nut_path2, act_nut_tcp, act_nut_fix1, pos_common).
//		action(act_nut_path3, act_nut_tcp, act_nut_fix1, pos_common).
//		action(act_nut_path4, act_nut_tcp, act_nut_fix1, pos_common).
		move(above_inside);

//	sequence("action_pih2").
//		move(outside_far).
//		action(act_nut_path, act_nut_tcp, act_nut_fix2, pos_common).
//		move(above_inside);

	sequence("pick_nuts").neverReversible().
		call("action_pih1").
//		call("fixat_nut");
//		call("action_pih2");


	// ******************************
	// * DROP ASSEMBLY PLATFORM

	sequence("drop_assembly").neverReversible().
		move(pos_drop_at).
		move(pos_drop_up).
		call("nut_release").
		call("tube_release").
		wait(1.0).
		move(pos_drop_at);


	 //******************************
	 //* MAIN SEQUENCE

	sequence("main");
		move( pos_common );
		call("pick_tube");
		move( pos_common );
//		call("bend_tube").
//		move( pos_common );
//		call("rotate_tube").
//		move( pos_common );
//		call("pick_nuts");
//		move( pos_common );
//		call("bend_tube").
//		move( pos_common );
//		call("drop_assembly").reverseWith("tube_grasp");
//		move( pos_common );

//	sequence("main");
//	move( pos_common );
//	call("action_pih1");
//	move( pos_common );

//		std::string filepath = dsl::common::getPath2dslOutputFolder() + "/iMoveSaves.xml";
//		dsl::intelligentmove::XMLRecordSaver saver;
//		dsl::intelligentmove::XMLRecordLoader loader;
//		dsl::intelligentmove::RecordDataset dataset;
//
//		dataset = loader.load(filepath);
//
//		JointConfiguration start (1.3265335, -1.550653, 2.2389111, -2.258156, 4.7054405, 1.3235337);
//
//		//Box test set
//		rw::math::Q q3(6, 1.3251073, -1.753568, 1.6261854, -1.442536, 4.7043156, 1.3272349);
//		rw::math::Q q4(6, 1.3265335, -1.550653, 2.2389111, -2.258156, 4.7054405, 1.3235337);
//		rw::math::Q q1(6, 2.3179018, -0.991876, 1.4687061, -2.045528, 4.7057037, 2.3164839);
//		rw::math::Q q2(6, 2.3184297, -1.076932, 0.8358640, -1.327704, 4.7055482, 2.3219072);
//
//		sequence("main").
//			move(start).
//			intMove(q1, dataset).
//			intMove(q2, dataset).
//			intMove(q3, dataset).
//			intMove(q4, dataset);
}

} /* namespace dsl */
