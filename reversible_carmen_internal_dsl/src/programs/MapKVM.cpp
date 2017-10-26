/*
 * MapKVM.cpp
 *
 *  Created on: May 19, 2015
 *      Author: josl
 */

#include "MapKVM.hpp"

namespace dsl {

MapKVM::MapKVM()
{
}

MapKVM::~MapKVM()
{
}

void MapKVM::buildExpandedModel()
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

	JointConfiguration pos_drop_at ( 2.0913, -1.278, 1.4849, -1.822, -1.600, 2.1096 );
	JointConfiguration pos_drop_up ( 2.3153, -1.369, 1.9475, -3.166, -2.232, 1.9490 );

	// ******************************
	// * PICK  TUBE

	//CRASH VALUES
	JointConfiguration pos_tube_take_at 	( 2.61505198, -1.2303603, 1.69783592, -2.0270512, -1.5772302, 2.60829782 );
	JointConfiguration pos_tube_take_in 	( 2.72028470, -1.2718833, 1.75522851, -2.0429990, -1.5760090, 2.71330547 );
	JointConfiguration pos_tube_take_above 	( 2.71761059, -1.3337281, 1.64773416, -1.8737214, -1.5764640, 2.71158146 );


/*
	JointConfiguration pos_tube_take_at 	( 2.649646, -1.261176, 1.736577, -2.038095, -1.578643, 2.659323 );
	JointConfiguration pos_tube_take_in 	( 2.714241, -1.284722, 1.768493, -2.046356, -1.578104, 2.723937 );
	JointConfiguration pos_tube_take_above 	( 2.713954, -1.356938, 1.636074, -1.841739, -1.578559, 2.725002 );
*/


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


	// ******************************
	// * ROTATE TUBE

	JointConfiguration pos_tube_rotate_at	( 2.07467913, -1.7643936, 2.15112400, -1.9333995, -1.5581501, 2.09054541 );
	JointConfiguration pos_tube_rotate_in	( 2.07476305, -1.7322056, 2.20173406, -2.0161636, -1.5578988, 2.09018635 );
	JointConfiguration pos_tube_rotate_out	( 2.02108335, -1.6431134, 2.12677145, -2.0293291, -1.5592396, 2.03641676 );

	sequence("rotate_tube").
		call("rotater_release").
		call("rotater_position_1").
		wait(0.5).
		move(pos_tube_rotate_at).
		move(pos_tube_rotate_in).
		uncall("rotater_release").
		wait(0.5).
		call("tube_release").
		wait(0.5).
		move(pos_tube_rotate_out).
		call("rotater_position_2").
		wait(0.5).
		move(pos_tube_rotate_in).
		uncall("tube_release").
		wait(0.5).
		call("rotater_release").
		move(pos_tube_rotate_at).
		uncall("rotater_release");


	// ******************************
	// * BEND TUBE

	//CRASH VALUES
/*
	JointConfiguration pos_machine_bend_before ( 2.66612124, -1.5517557, 1.94304323, -1.9524143, -1.5480473, 2.68197417 );
	JointConfiguration pos_machine_bend_at ( 2.74560999, -1.4740241, 1.87415647, -1.9635880, -1.5473530, 2.76173329 );
	JointConfiguration pos_machine_bend_in ( 2.86369013, -1.4906099, 1.89352703, -1.9698708, -1.5467546, 2.87956881 );
*/

	JointConfiguration pos_machine_bend_before ( 2.707035, -1.486790, 1.879281, -1.954800, -1.547605, 2.723254 );
	JointConfiguration pos_machine_bend_at ( 2.836421, -1.509902, 1.905499, -1.961670, -1.546862, 2.852618 );
	JointConfiguration pos_machine_bend_in ( 2.875262, -1.499520, 1.894533, -1.962389, -1.546779, 2.891434 );

	sequence("bend_tube").
//		call("machine_open").
		wait(0.5).
		move(pos_machine_bend_before).
		move(pos_machine_bend_at).
		move(pos_machine_bend_in).
		uncall("machine_open").
		wait(2.0).
		call("machine_open").
		wait(0.5).
		move(pos_machine_bend_in).
		move(pos_machine_bend_at).
		move(pos_machine_bend_before);
//		uncall("machine_open");


	// ******************************
	// * NUT GRASP AND DROP

	JointConfiguration nutGrasp_at 			(2.499607, -1.66887, 2.016528, -1.87577, -1.53667, 2.493259);
	JointConfiguration nutGrasp_angle_up 	(2.386445, -1.55364, 1.832422, -1.05189, -0.93596, 2.103391);
	JointConfiguration nutGrasp_angle_down 	(2.625256, -1.62003, 2.158690, -2.53937, -2.22100, 2.474643);

	sequence("drop_nut").
		move(nutGrasp_at).
		move(nutGrasp_angle_down).		neverReversible().
		wait(1.0).						neverReversible().
		move(nutGrasp_angle_down).		neverReversible();

	sequence("fixat_nut").
		move(nutGrasp_at).
		move(nutGrasp_angle_up).
		call("nut_grasp");
		move(nutGrasp_at);


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

	JointConfiguration outside_far 		(2.563623, -1.55670, 2.234624, -2.23731, -1.57708, 2.555370);
	JointConfiguration above_inside 	(2.680894, -1.64686, 2.216617, -2.12918, -1.57614, 2.673222);
	JointConfiguration above_tilted 	(2.517352, -1.36697, 1.685097, -1.31548, -0.92325, 2.340487);

	std::string act_nut_pathfile		= "/../../../../scenes/KVM_pih_legacy/Trajectories/FemaleTmale_TCP_Good1453.txt"; //8
	std::string act_nut_basepath 		= ros::package::getPath("reversible_dsl_v1");
	std::string act_nut_path 			= act_nut_basepath + act_nut_pathfile;
	std::string act_nut_tcp				= "TCP_KVM";
	std::string act_nut_fix1			= "HoleACP1";
	std::string act_nut_fix2			= "HoleACP2";

	sequence("action_pih1").
		move(outside_far).
		action(act_nut_path, act_nut_tcp, act_nut_fix1, pos_common).
		move(above_inside);


	sequence("action_pih2").
		move(outside_far).
		action(act_nut_path, act_nut_tcp, act_nut_fix2, pos_common).
		move(above_inside);

	sequence("pick_nuts").
		call("action_pih2").reverseWith("drop_nut").
		call("fixat_nut");
		call("action_pih1").reverseWith("drop_nut");
		move(above_tilted);


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
//		move( pos_common );
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



}


} /* namespace dsl */
