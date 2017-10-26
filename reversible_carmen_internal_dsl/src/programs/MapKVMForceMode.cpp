/*
 * MapKVMForceMode.cpp
 *
 *  Created on: Jun 10, 2015
 *      Author: josl
 */

#include "MapKVMForceMode.hpp"

namespace dsl {

MapKVMForceMode::MapKVMForceMode()
{
}

MapKVMForceMode::~MapKVMForceMode()
{
}

void MapKVMForceMode::buildExpandedModel()
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
	JointConfiguration pos_common( 0.7435056210574957, -1.6462719783681372, 1.909417165656409, -1.8426958594885843, -1.5612201638921421, -3.968951914581704 );

	sequence("go_kvm_from_vola").
			move(pos_screwdriver_take_infront).
			move(pos_common);



	// ******************************
	// * ALTERNATIVE DROP

	JointConfiguration pos_drop_at (0.6723065739043981, -1.3345095963381006, 1.5001166919294642, -1.1794124540458613, -2.1570521453849327, -2.297196258661514  );
	JointConfiguration pos_drop_up ( 0.7058110791988592, -1.0942307509831826, 1.2032846088255154, -0.10910237032775713, -2.4487270034690596, -1.570724133611515 );

	// ******************************
	// * PICK  TUBE

	JointConfiguration pos_tube_take_at 	( 1.0827117502873247, -1.2271445744403877, 1.7717438727918058, -2.1266620806180416, -1.564588593954893, -3.6297109128399025 );
	JointConfiguration pos_tube_take_in 	( 1.1462446070128203, -1.2492939659169537, 1.8031546860584013, -2.1363461013324008, -1.5653339288544652, -3.566135067098889 );
	JointConfiguration pos_tube_take_above 	( 1.1462446070128203, -1.3205693925301147, 1.7114209731303456, -1.973343181596512, -1.565325384102107, -3.5662160804592533 );

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

	JointConfiguration pos_tube_rotate_at	( 0.7195260690608369, -1.9501813822415661, 2.3583698986561816, -1.987383132505867, -1.5610018712354812, -3.992808180064154 );
	JointConfiguration pos_tube_rotate_in	( 0.7195072138803191, -1.9220247686015624, 2.3968708158654897, -2.053996152349006, -1.5610180739075545, -3.992808180064154 );
	JointConfiguration pos_tube_rotate_out	( 0.6327128104322809, -1.8502627057582046, 2.3521244237100394, -2.080072878990471, -1.5603213470242006, -4.079605810468424);

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

	JointConfiguration pos_machine_bend_before ( 1.150875190076103, -1.485609228621508, 1.9063247434786579, -2.003204514479034, -1.5654064094466964, -3.561507909925823 );
	JointConfiguration pos_machine_bend_at ( 1.2933359219311371, -1.483114816070741, 1.9411039420667455, -2.0410818677008367, -1.5670906244780713, -3.4190822998327817 );
	JointConfiguration pos_machine_bend_in ( 1.317033956495286, -1.4641857743780486, 1.9245979636434232, -2.0436811022722274, -1.567414677919527, -3.3954195915667866 );

	sequence("bend_tube").
		call("machine_open").
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
		move(pos_machine_bend_before).
		uncall("machine_open");


	// ******************************
	// * NUT GRASP AND DROP

	JointConfiguration nutGrasp_at 			(0.9828526178248799, -1.649317262594593, 2.137432991489362, -2.069566836321367, -1.5635115476941497, -3.7295593401982448);
	JointConfiguration nutGrasp_angle_up 	(0.8812613973473811, -1.603850564193845, 2.0227751245214716, -1.509515148738619, -1.0425085991290102, -3.958656398791522);
	JointConfiguration nutGrasp_angle_down 	(1.262550602113974, -1.6620762500632145, 2.229853870286111, -2.3237251142902573, -2.063365898615179, -3.4929527745313336);


	sequence("fixat_nut").
		move(nutGrasp_at).
		move(nutGrasp_angle_up).
		call("nut_grasp");
		move(nutGrasp_at);

	sequence("drop_nut").
		move(nutGrasp_at).
		move(nutGrasp_angle_down).		neverReversible().
		wait(1.0).						neverReversible().
		move(nutGrasp_angle_down).		neverReversible();


	// ******************************
	// * FORCE MODE PICKUP

	JointConfiguration outside_farUP 	(0.9773977629684002, -1.6960783148093999, 2.044482914887139, -1.9296525445258057, -1.5635200924465078, -3.734964237615029);
	JointConfiguration outside_far 		(1.0290378711761934, -1.5245435657108464, 2.298115555152767, -2.355282538571036, -1.5640300451847038, -3.68342017018434);
	JointConfiguration outside_close 	(1.0546417426459707, -1.5315042400245804, 2.305532818848015, -2.3559630627823176, -1.5642807431855097, -3.657778159317112);
	JointConfiguration inside_deep 		(1.1067397505105343, -1.5431533732184066, 2.3178999573159573, -2.3570077036989145, -1.5648964567085004, -3.6057565564765977);
	JointConfiguration inside_backplate (1.1269751174590166, -1.5451346372376928, 2.3215284546463812, -2.359250479484704, -1.569224188022293, -3.5870126534809788);
	JointConfiguration above_inside 	(1.1067397505105343, -1.6403470478823836, 2.249989848062712, -2.1919148310132366, -1.564880254036428, -3.605724151132452);

	dsl::ForceModeArgument deactivate = dsl::ForceModeArgument::getDeativationArgument();
	dsl::ForceModeArgument forward = dsl::ForceModeArgument::getPushForward(30);
	dsl::ForceModeArgument backward = dsl::ForceModeArgument::getPushForward(-30);

	dsl::PositionJointComparison * inside_nut = new dsl::PositionJointComparison(inside_backplate, 0.03, _urrtInterface);

	sequence("pick_nut").reverseWith("drop_nut").
		move(outside_farUP).
		move(outside_far).
		move(outside_close).
		forceMode(forward,deactivate).
		wait(2).
		forceMode(deactivate,deactivate).
		check(inside_nut);
		move(inside_deep).
		move(above_inside);

	sequence("pick_nuts").
		call("pick_nut").
		call("fixat_nut");
		call("pick_nut");


	// ******************************
	// * DROP ASSEMBLY PLATFORM

	sequence("drop_assembly").neverReversible().
		move(pos_drop_at).
		move(pos_drop_up).
		call("nut_release").
		call("tube_release").
		wait(1.0).
		move(pos_drop_at);


	// ******************************
	// * MAIN SEQUENCE

	sequence("main_seq").
		move( pos_common );
		call("pick_tube");
		move( pos_common );
		call("bend_tube").
		move( pos_common );
		call("rotate_tube").
		move( pos_common );
		call("pick_nuts");
		move( pos_common );
		call("bend_tube").
		move( pos_common );
		call("drop_assembly").reverseWith("tube_grasp");
		move( pos_common );


		sequence("main");
		for(int i = 0; i < 20; i++)
		{
			call("main_seq");
			print("############################################################");
			print(std::to_string(i));
			print("############################################################");
		}


}

} /* namespace dsl */

