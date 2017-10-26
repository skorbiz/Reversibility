/*
 * MapVola.cpp
 *
 *  Created on: May 19, 2015
 *      Author: josl
 */

#include "MapVola.hpp"

namespace dsl {

MapVola::MapVola()
{
}

MapVola::~MapVola()
{
}

void MapVola::buildExpandedModel()
{

	// ******************************
	// * COMMON

	sequence("skip").
		print("skip");


	// ******************************
	// * PICK BRACKETS FRONT

	JointConfiguration pos_bracket_common(0.3167593052700785, 1.490484935423108, 1.7819021747989927, -1.8620596944543601, -1.570973225726184, -1.2536904758631362);
	JointConfiguration pos_bracket_front_above_1b(0.6822396683571638, -1.2733815421190127, 1.6775078217455786, -1.9749139779233778, -1.5706653629725764, -0.8885511806044937);
	JointConfiguration pos_bracket_front_at_1b(0.6826262059493643, -1.1821548226245258, 1.7812771926764883, -2.169863198054016, -1.570762579005013, -0.888194721818893);
	JointConfiguration pos_bracket_front_above_2b(0.5643070528117096, -1.4283667796960664, 1.8015100093819107, -1.9439841435324707, -1.5704385255635573, -1.0065129729492561);
	JointConfiguration pos_bracket_front_at_2b(0.5646410835198582, -1.3011689713244785, 1.9516811230944109, -2.2213395665982123, -1.5704709309077032, -1.0061240968352845);
	JointConfiguration pos_bracket_front_above_3b(0.4213795114281622, -1.5509423715912523, 1.9356720010482853, -1.9555220759028717, -1.570389917547339, -1.1493609071279813);
	JointConfiguration pos_bracket_front_at_3b(0.4216475426129121, -1.411441915860679, 2.0943366449431604, -2.2536423257785634, -1.5704061202194124, -1.1490044483423798);
	JointConfiguration pos_bracket_front_above_4b(0.2511348657403014, -1.6586551661100728, 2.03918832906506, -1.9512224116438093, -1.5702602961707575, -1.3193899262574744);
	JointConfiguration pos_bracket_front_at_4b(0.25132341754547893, -1.5093398602773993, 2.2074604096830193, -2.2687874139718405, -1.5703089041869758, -1.3192116968646737);

	sequence("pick_bracket_front_1").
		move(pos_bracket_common).
		gripper_open().
		move(pos_bracket_front_above_1b).
		move(pos_bracket_front_at_1b).
		gripper_close().
		move(pos_bracket_front_above_1b).
		move(pos_bracket_common);

	sequence("pick_bracket_front_2").
		move(pos_bracket_common).
		gripper_open().
		move(pos_bracket_front_above_2b).
		move(pos_bracket_front_at_2b).
		gripper_close().
		move(pos_bracket_front_above_2b).
		move(pos_bracket_common);

	sequence("pick_bracket_front_3").
		move(pos_bracket_common).
		gripper_open().
		move(pos_bracket_front_above_3b).
		move(pos_bracket_front_at_3b).
		gripper_close().
		move(pos_bracket_front_above_3b).
		move(pos_bracket_common);

	sequence("pick_bracket_front_4").
		move(pos_bracket_common).
		gripper_open().
		move(pos_bracket_front_above_4b).
		move(pos_bracket_front_at_4b).
		gripper_close().
		move(pos_bracket_front_above_4b).
		move(pos_bracket_common);


	// ******************************
	// * PICK BRACKETS BACK

	JointConfiguration pos_bracket_back_above_1a(0.4769617840795684, -1.2042570761497975, 1.519511744202343, -1.8860505667509537, -1.570940820382039, -1.0938153669223372);
	JointConfiguration pos_bracket_back_at_1a(0.47734295273901145, -1.1316495853615431, 1.628318343720231, -2.067453681912159, -1.570835934445662, -1.0934589081367365);
	JointConfiguration pos_bracket_back_above_2a(0.3543193042304443, -1.3060729608968553, 1.6576144046810617, -1.9223494537419512, -1.5708922123658207, -1.2163718991117953);
	JointConfiguration pos_bracket_back_at_2a(0.35477589361195827, -1.2259633644926098, 1.7676870688506447, -2.1125087110041774, -1.570762579005013, -1.2158210082613214);
	JointConfiguration pos_bracket_back_above_3a(0.21595484436392615, -1.38984876499256, 1.7628585000003385, -1.9438545221558883, -1.5708111870212313, -1.354518661544435);
	JointConfiguration pos_bracket_back_at_3a(0.2164962820577701, -1.3037482385791617, 1.875357033345816, -2.1424390730221567, -1.5707139709887947, -1.3540411141503848);
	JointConfiguration pos_bracket_back_above_4a(0.06419312679770771, -1.452221472048178, 1.8361870065000057, -1.9547852857556576, -1.570762579005013, -1.5062321418133457);
	JointConfiguration pos_bracket_back_at_4a(0.06484769557465818, -1.3618781465230878, 1.9513470987778492, -2.1602039127798633, -1.570681565644649, -1.5056411757147865);


	sequence("pick_bracket_back_1").
		move(pos_bracket_common).
		gripper_open().
		move(pos_bracket_back_above_1a).
		move(pos_bracket_back_at_1a).
		gripper_close().
		move(pos_bracket_back_above_1a).
		move(pos_bracket_common);

	sequence("pick_bracket_back_2").
		move(pos_bracket_common).
		gripper_open().
		move(pos_bracket_back_above_2a).
		move(pos_bracket_back_at_2a).
		gripper_close().
		move(pos_bracket_back_above_2a).
		move(pos_bracket_common);

	sequence("pick_bracket_back_3").
		move(pos_bracket_common).
		gripper_open().
		move(pos_bracket_back_above_3a).
		move(pos_bracket_back_at_3a).
		gripper_close().
		move(pos_bracket_back_above_3a).
		move(pos_bracket_common);

	sequence("pick_bracket_back_4").
		move(pos_bracket_common).
		gripper_open().
		move(pos_bracket_back_above_4a).
		move(pos_bracket_back_at_4a).
		gripper_close().
		move(pos_bracket_back_above_4a).
		move(pos_bracket_common);



	// ******************************
	// * PLACE BACK BRACKET

	JointConfiguration pos_fixture_behind_free(-0.4008563824687803, -1.8279920658835298, 1.6535453608515311, -1.443788853680492, -1.5509938929534668, -0.12544089515641144);
	JointConfiguration pos_fixture_behind_before_1(-0.8037954634347111, -1.8672792888615346, 1.9650569689789474, -0.13434441967297575, -0.8055981015342608, 0.024774928226836565);
	JointConfiguration pos_fixture_behind_before_2(-0.8977614635548177, -1.6642177831169018, 2.2366502105163564, -0.6060402522599116, -0.8995274962431301, 0.020402807343992465);
	JointConfiguration pos_fixture_behind_after_1(-1.0045027496894168, -1.579348167628809, 2.1816715976545025, -0.6335651406547799, -1.0062218042209423, 0.016115127309835314);
	JointConfiguration pos_fixture_behind_after_2(-1.004493322099158, -1.7149238120734323, 1.9587832112620278, -0.2750208961883267, -1.006173196204724, 0.016115127309835314);

	std::string act_basepath 			= ros::package::getPath("sdu_magic");
	std::string act_back_pathfile		= "/../../../scenes/TrajectoryLmount.txt";
	std::string act_back_path 			= act_basepath + act_back_pathfile;
	std::string act_back_tcp			= "Lmount_TCP";
	std::string act_back_fix			= "Lmount";

	sequence("action_place_back").
		action( act_back_path, act_back_tcp , act_back_fix , pos_fixture_behind_before_2);

	sequence("place_bracket_back").
		move(pos_fixture_behind_free);
		move(pos_fixture_behind_before_1);
		move(pos_fixture_behind_before_2);
		call("action_place_back");
		gripper_open();
		move(pos_fixture_behind_after_1);
		move(pos_fixture_behind_after_2);
		move(pos_fixture_behind_free);


	// ******************************
	// * PLACE FRONT BRACKET

	JointConfiguration pos_fixture_forward_free (0.880950286868838, -1.162529443136346, 1.704013143416943, -0.5403435941899769, -2.2208856998983215, -0.039050081250796786);
	JointConfiguration pos_fixture_forward_before (0.7852839968770932, -1.2669097986150708, 1.9354039634719484, -0.6672597956526016, -2.3166364810768396, -0.03887185185799602 );
	JointConfiguration pos_fixture_forward_after (0.8096878032185747, -1.1304912892517842, 1.8422775749529174, -0.7105166157659392, -2.3143894868438837, -0.03887950977771126);

	std::string act_front_pathfile		= "/../../../scenes/TrajectoryVmount.txt";
	std::string act_front_path 			= act_basepath + act_front_pathfile;
	std::string act_front_tcp			= "Vmount_TCP";
	std::string act_front_fix			= "Vmount";

	sequence("action_place_front").
			action( act_front_path, act_front_tcp, act_front_fix, pos_fixture_forward_free);

	sequence("place_bracket_front").
		move(pos_fixture_forward_free).
		move(pos_fixture_forward_before).
		nest("action_place_front").
		gripper_open().
		move(pos_fixture_forward_after).
		move(pos_fixture_forward_free);


	// ******************************
	// * MOVETO SEQUENCES

	JointConfiguration pos_screwdriver_take_infront(0.849006658158743, -0.9276855492371849, 1.4052383056030697, -0.4775813335093756, -3.8633762620935084, -1.5707727416277333);
	JointConfiguration pos_screwdriver_to_bracket_via_1(0.8675058912178433, -1.3288715522176142, 1.7749252019393875, -1.0274155708263757, -3.9798765302740966, -1.4952971597664098);
	JointConfiguration pos_screwdriver_to_bracket_via_2(0.6062806085118054, -1.5178603629552943, 1.7757509757554386, -1.9436438754347174, -2.508393187582122, -1.4948434729641473);

	sequence("from_bracket_to_screwdriver").
		move(pos_bracket_common).
		move(pos_screwdriver_to_bracket_via_2).
		move(pos_screwdriver_to_bracket_via_1).
		move(pos_screwdriver_take_infront);

	// ******************************
	// * SCREWDRIVER PICK AND PLACE

	JointConfiguration pos_screwdriver_take_in (0.7438638758789198, -0.8092334510979134, 1.195523893223446, -0.38633910163003055, -3.9685696657441056, -1.5708537549880974);
	JointConfiguration pos_screwdriver_take_above(0.7456188713404062, -0.8657872321318071, 1.1150708385207926, -0.24927743885727027, -3.96676003579909, -1.5707079309394425);

	IOPorts screwdriver_holder(0);
	macro("screwdriver_holder_open"). io(screwdriver_holder, Switch::on);
	macro("screwdriver_holder_close").io(screwdriver_holder, Switch::off);

	sequence("pick_screwdriver").
		gripper_open().
		move( pos_screwdriver_take_infront ).
		move( pos_screwdriver_take_in ).
		gripper_close().
		call("screwdriver_holder_open").
		move( pos_screwdriver_take_above).
		call( "screwdriver_holder_close" );


	// ******************************
	// * LOCK, UNLOCK AND REMOVE

	JointConfiguration lockUnlockCommon (-0.6863347834702367, -2.2891993495530008, 2.164532455877808, -1.4461288454987178, -1.5707873264294436, -0.6863681145415068);

	JointConfiguration drop_1 (-0.8983823870198204, -1.6354053479489714, 2.234367263608786, -0.5988138605154549, -0.8984828433423084, 1.8395785230552065e-05);
	JointConfiguration drop_2 (-0.8986086491860332, -1.7742486782679467, 1.9994093629791516, -0.22518157276627654, -0.8986286673909634, 1.8395785230552065e-05);
	JointConfiguration drop_3 (-1.1426344918872395, -1.4985756230758243, 1.7437946661374975, -0.24526943467974202, -1.142628073702049, 1.8395785230552065e-05);
	JointConfiguration drop_4 (-1.1358570068375116, -1.6172546928491576, 1.803403862062634, -0.6253265013536465, -1.172720354582732, 0.18015260922917528);

	sequence("remove_finished_bracket").neverReversible().
		move(pos_fixture_behind_free);
		move(pos_fixture_behind_after_1);
		gripper_open();
		move(drop_1);
		gripper_close();
		move(drop_2);
		move(drop_3);
		move(drop_4);
		gripper_open();
		move(pos_fixture_behind_free);



	JointConfiguration unlock_at_1 (-0.3897838406951868, -2.315095930123453, 2.2410316175754073, -1.4966085100259399, -1.5707139709887947, -0.3897852010367444);
	JointConfiguration unlock_at_2 (-0.3178395537174019, -2.307479178618327, 2.238969563901292, -1.5022683358749227, -1.5706977683167223, -0.3179033286979518);
	JointConfiguration unlock_at_3 (-0.3178395537174019, -2.307041444417331, 2.2336561228986955, -1.4973453001731545, -1.5707139709887947, -0.3178061126655152);
	JointConfiguration unlock_at_4 (-0.16848564909488672, -2.281082929372568, 2.2264016912640816, -1.5160047503358611, -1.570681565644649, -0.16857787302069038);
	JointConfiguration unlock_at_5 (-0.1987119863129445, -2.3841521019896743, 2.252061329327085, -1.438615131962142, -1.570681565644649, -0.19866672641304994);
	JointConfiguration unlock_at_6 (-0.1987119863129445, -2.3866748037028045, 2.269698829503831, -1.4537765187012914, -1.570681565644649, -0.19861811839683163);
	JointConfiguration unlock_at_7 (-0.246504205979714, -2.410596991209583, 2.274400774631684, -1.4344979155430466, -1.5706977683167223, -0.24645128256963478);
	JointConfiguration unlock_at_8 (-0.24649477838945444, -2.3982782945766044, 2.207003820301505, -1.3794516860838821, -1.5706977683167223, -0.24646748524170814);

	sequence("unlock_fixture").
		move(pos_fixture_behind_free);
		move(lockUnlockCommon);
		gripper_close();
		move(unlock_at_1);
		move(unlock_at_2);
		move(unlock_at_3);
		move(unlock_at_4);
		move(unlock_at_5);
		move(unlock_at_6);
		move(unlock_at_7);
		move(unlock_at_8);
		move(lockUnlockCommon);
		move(pos_fixture_behind_free);


	JointConfiguration lock_1 (-0.15855256103370774, -2.377148348382147, 2.2203016888293243, -1.4138063363157216, -1.570576691692498, -0.15854171982592113);
	JointConfiguration lock_2 (-0.15854312705186224, -2.3865333834573343, 2.2837873181213126, -1.4679051408907253, -1.570576691692498, -0.15854171982592113);
	JointConfiguration lock_3 (-0.4667694724417313, -2.4918964325951247, 2.3009493737360267, -1.379678535477126, -1.5706252997087162, -0.4667921870776679);
	JointConfiguration lock_4 (-0.36580147640041716, -2.317769473280725, 2.2676367758297165, -1.5204903019074396, -1.570616754956358, -0.3745386783638631);
	JointConfiguration lock_5 (-0.3449938259610814, -2.315491888914326, 2.267001062233284, -1.5221421115946692, -1.5706005522842856, -0.35361957862669957);

	sequence("lock_fixture").
		move(pos_fixture_behind_free);
		move(lockUnlockCommon);
		gripper_close();
		move(lock_1);
		move(lock_2);
		move(lock_3);
		move(lock_4);
		move(lock_5);
		move(lockUnlockCommon);
		move(pos_fixture_behind_free);

	sequence("zoned_lock").
		gripper_close();
		print("skip").reverseWith("unlock_fixture").
		print("skip").
		call("lock_fixture").reverseWith("skip");
		gripper_close();

	// ******************************
	// * VOLA TO KVM

	JointConfiguration pos_screwdriver(0.849006658158743, -0.9276855492371849, 1.4052383056030697, -0.4775813335093756, -3.8633762620935084, -1.5707727416277333);
	JointConfiguration pos_common( 0.7435056210574957, -1.6462719783681372, 1.909417165656409, -1.8426958594885843, -1.5612201638921421, -3.968951914581704 );

	sequence("go_kvm_from_vola").
			move(pos_screwdriver).
			move(pos_common);


	// ******************************
	// FORCE MODE OPERATIONS

	boost::array<double,6> taskframe 			= 		{{    0,     0,    0,     0,     0,    0 }};
	ForceModeArgument::forcemodeType_t type 	= 		ForceModeArgument::forcemodeType_t::FRAME;

	boost::array<double,6> wrench_down 			= 		{{    0,     0,  -40,     0,     0,    0 }};
	boost::array<double,6> wrench_up_mild 		=		{{    0,     0,  -10,     0,     0,    0 }};
	boost::array<double,6> wrench_up_strong 	= 		{{ 	  0,     0,    5,     0,     0,    0 }};
	boost::array<double,6> wrench_left 			= 		{{  100,   -1,     0,     0,     0,    0 }};
	boost::array<double,6> wrench_right 		=		{{  100,   55,     0,     0,     0,    0 }};

	boost::array<double,6> limit_ud				= 		{{  100,   100,  0.02,    10,    10,   10 }};
	boost::array<double,6> limit_lr 			= 		{{  100,  0.02,   100,    10,    10,   10 }};
	boost::array<bool,6> selectionvector_ud 	= 		{{false, false, true, false, false, false}};
	boost::array<bool,6> selectionvector_lr 	= 		{{false, true, false, false, false, false}};

	dsl::ForceModeArgument fma_down	 	(type, taskframe, wrench_down, limit_ud, selectionvector_ud);
	dsl::ForceModeArgument fma_up_mild 	(type, taskframe, wrench_up_mild, limit_ud, selectionvector_ud);
	dsl::ForceModeArgument fma_up_strong(type, taskframe, wrench_up_strong, limit_ud, selectionvector_ud);
	dsl::ForceModeArgument fma_left 	(type, taskframe, wrench_left, limit_lr, selectionvector_lr);
	dsl::ForceModeArgument fma_right 	(type, taskframe, wrench_right, limit_lr, selectionvector_lr);
	dsl::ForceModeArgument fma_deativate = dsl::ForceModeArgument::getDeativationArgument();

	// ******************************
	// SCREWING INSERT / REMOVE

	IOPorts screwdriver_activate(5);
	IOPorts screwdriver_set_backwards(4);

	IoComparison * scrwing_finished = new IoComparison(IOPorts(4),  Switch(Switch::on), _pDigitalIO);
	IoComparisonMonitored * screwing_succeded = new IoComparisonMonitored(IOPorts(5), Switch(Switch::on), _pDigitalIO);

	sequence("wait_for_screwing_backward").neverReversible().
		wait(1.7).
		io( screwdriver_set_backwards, Switch::off);

	sequence("wait_for_screwing").
		wait(0.3).
		wait(scrwing_finished).reverseWith("wait_for_screwing_backward").
		wait(0.3);

	sequence("screw_insert").
		wait(0.2).
		io(screwdriver_activate, Switch::on).
//		call("wait_for_screwing").
		wait(0.3).
		wait(scrwing_finished).reverseWith("wait_for_screwing_backward").
		wait(0.3);
		io(screwdriver_activate,Switch::off).
		io(screwdriver_set_backwards, Switch::off);


	// ******************************
	// SCREWING WITH DIRECTIONS

	sequence("screw_insert_nonReversible").
//		check_start( screwing_succeded ).
		call("screw_insert").neverReversible().reverseWith("skip").
//		check_stop( screwing_succeded );

	sequence("screw_in_table").
		check_start( screwing_succeded ).
		forceMode( fma_down , fma_deativate).
		wait(0.2).
		io(screwdriver_activate, Switch::on).
//		call("wait_for_screwing").
		wait(0.3).
		wait(scrwing_finished).reverseWith("wait_for_screwing_backward").
		wait(0.3);
		io(screwdriver_activate,Switch::off).
		io(screwdriver_set_backwards, Switch::off);
		forceMode( fma_deativate , fma_up_mild).
		check_stop( screwing_succeded ).
		print("skip").reverseWith("screw_insert_nonReversible").
		forceMode( fma_deativate , fma_down);

//	sequence("screw_in_fixture").
//		check_start( screwing_succeded ).
//		forceMode( fma_right , fma_deativate).
//		wait(0.2).
//		io(screwdriver_activate, Switch::on).
////		call("wait_for_screwing").
//		wait(0.3).
//		wait(scrwing_finished).reverseWith("wait_for_screwing_backward").
//		wait(0.3);
//		io(screwdriver_activate,Switch::off).
//		io(screwdriver_set_backwards, Switch::off);		forceMode( fma_deativate , fma_left).
//		check_stop( screwing_succeded ).
//		print("skip").reverseWith("screw_insert_nonReversible").
//		forceMode( fma_deativate , fma_right);

	// ******************************
	// SCREWING IN TABLE

	JointConfiguration pos_screw_table_common (1.096768945696733, -1.044318154414426, 1.424346471510681, -0.38035945277098415, -3.615565021382379, -1.5711454150696325);
	JointConfiguration pos_screw_at_t1a	(1.025833621799006, -0.895316170550999, 1.3650124333893716, -0.4697077097304261, -3.68658611478308, -1.570878502412528);
	JointConfiguration pos_screw_at_t1b (0.990865072457414, -0.9733176363332139, 1.5000789815684286, -0.5267530037776933, -3.721483997945679, -1.5709347683484607);
	JointConfiguration pos_screw_at_t2a	(1.0829568676340555, -0.9240570519067614, 1.4153102572750722, -0.4915565815884637, -3.629410528242636, -1.571161617741705);
	JointConfiguration pos_screw_at_t2b (1.050650931836622, -1.0024842612734806, 1.5495512873015427, -0.5473107431812693, -3.661684429409414, -1.5710968070534141);
	JointConfiguration pos_screw_at_t3a	(1.1431779109712, -0.9473257188567945, 1.4555766585395045, -0.5090051336824537, -3.569210219209743, -1.5717039638398216);
	JointConfiguration pos_screw_at_t3b (1.1139967387797947, -1.026287501353332, 1.5893799479733914, -0.563714654358662, -3.598362948842029, -1.5716715584956757);
	JointConfiguration pos_screw_at_t4a	(1.2060979679382817, -0.9650720421872676, 1.4860793999207784, -0.522255467981209, -3.506293805444177, -1.572303474690739);
	JointConfiguration pos_screw_at_t4b (1.1806489965701479, -1.0446144747615391, 1.6196429409406923, -0.5761957931660572, -3.531706318404457, -1.5723682853790297);

	sequence("pick_screw_1a_table").
			move(pos_screw_table_common).
			move(pos_screw_at_t1a).
			uncall("screw_in_table").
			move(pos_screw_at_t1a).
			move(pos_screw_table_common);

	sequence("pick_screw_1b_table").
			move(pos_screw_table_common).
			move(pos_screw_at_t1b).
			uncall("screw_in_table");
			move(pos_screw_at_t1b).
			move(pos_screw_table_common);

	sequence("pick_screw_2a_table").
			move(pos_screw_table_common).
			move(pos_screw_at_t2a).
			uncall("screw_in_table").
			move(pos_screw_at_t2a).
			move(pos_screw_table_common);

	sequence("pick_screw_2b_table").
			move(pos_screw_table_common).
			move(pos_screw_at_t2b).
			uncall("screw_in_table");
			move(pos_screw_at_t2b).
			move(pos_screw_table_common);

	sequence("pick_screw_3a_table").
			move(pos_screw_table_common).
			move(pos_screw_at_t3a).
			uncall("screw_in_table").
			move(pos_screw_at_t3a).
			move(pos_screw_table_common);

	sequence("pick_screw_3b_table").
			move(pos_screw_table_common).
			move(pos_screw_at_t3b).
			uncall("screw_in_table");
			move(pos_screw_at_t3b).
			move(pos_screw_table_common);

	sequence("pick_screw_4a_table").
			move(pos_screw_table_common).
			move(pos_screw_at_t4a).
			uncall("screw_in_table").
			move(pos_screw_at_t4a).
			move(pos_screw_table_common);

	sequence("pick_screw_4b_table").
			move(pos_screw_table_common).
			move(pos_screw_at_t4b).
			uncall("screw_in_table");
			move(pos_screw_at_t4b).
			move(pos_screw_table_common);

	// ******************************
	// * SCREWING IN FIXTURE ACTIONS

	JointConfiguration pos_screw_fixture_common(0.6265779099345986, -1.285397041057679, 1.5927727491557562, 1.2634165853245083, -4.712390334531254, -2.515049513716255);
	JointConfiguration pos_screw_fixture_at_1 (0.5308079228415926, -1.4421589479664345, 1.8009025721646683, 1.2120331065075582, -4.712390334531254, -2.610814592075086);
	JointConfiguration pos_screw_fixture_at_2 (0.45145218851237995, -1.338688958952627, 1.6555037110327984, 1.2539994652830289, -4.712390334531254, -2.690005295641462);
	JointConfiguration pos_screw_fixture_inside_manual_1 (0.5217543554483208, -1.4470063656056715, 1.8101624919318438, 1.207531352263907, -4.712293106514593, -2.619810442642673);
	JointConfiguration pos_screw_fixture_inside_manual_2 (0.43797951009060854, -1.346289405520295, 1.6686601593151469, 1.2483234487461983, -4.712293106514593, -2.703296580036721);

	JointConfiguration pos_screw_fixture_on_screwdriver_check_1 (0.5325723458933379, -1.4352871806225318, 1.8116736803218556, 1.2104137021644743, -4.6863552291028885, -2.611713408943028);
	JointConfiguration pos_screw_fixture_on_screwdriver_check_2 (0.4473174072145015, -1.334156710449434, 1.669008986546312, 1.2501859051545987, -4.683064360943729, -2.6948635086707142);//0.45452039296844177, -1.3275677772048844, 1.6633372971138778, 1.2501372851541555, -4.682464862077037, -2.679743072028068);
	JointConfiguration pos_screw_fixture_inside_check_1 (0.5302570511659032, -1.435781486756668, 1.8132737756816981, 1.21135346912892, -4.68527817085792, -2.6122395643532963);
	JointConfiguration pos_screw_fixture_inside_check_2 (0.4444741674325696, -1.336156829649238, 1.6734846695079515, 1.2505099585960542, -4.683363678944978, -2.695527830209923);

	std::string act_screw_pathfile		= "/../../../scenes/TrajectoryOptimalVOlaPiH.txt";
	std::string act_screw_path 			= act_basepath + act_screw_pathfile;
	std::string act_screw_tcp			= "TCP_Screwdriver";
	std::string act_screw_fix2			=  "hole2";
	std::string act_screw_fix1			=  "hole1";

	dsl::PositionJointComparison * screw_1_on = new dsl::PositionJointComparison(pos_screw_fixture_on_screwdriver_check_1, 0.03, _urrtInterface);
	dsl::PositionJointComparison * screw_2_on = new dsl::PositionJointComparison(pos_screw_fixture_on_screwdriver_check_2, 0.023, _urrtInterface);

	dsl::PositionJointComparison * screw_1_inserted = new dsl::PositionJointComparison(pos_screw_fixture_inside_check_1, 0.023, _urrtInterface);
	dsl::PositionJointComparison * screw_2_inserted = new dsl::PositionJointComparison(pos_screw_fixture_inside_check_2, 0.023, _urrtInterface);

//	sequence("action_screw_1").neverReversible().action(act_screw_path, act_screw_tcp, act_screw_fix1, pos_screw_fixture_common);
//	sequence("action_screw_2").neverReversible().action(act_screw_path, act_screw_tcp, act_screw_fix2, pos_screw_fixture_common);

	sequence("place_screw_manual_1").
		move(pos_screw_fixture_common).
		move(pos_screw_fixture_at_1);

	sequence("place_screw_manual_2").
		move(pos_screw_fixture_common).
		move(pos_screw_fixture_at_2);

	sequence("move_pos_screw_fixture_inside_1").
		move(pos_screw_fixture_inside_manual_1);

	sequence("move_pos_screw_fixture_inside_2").
		move(pos_screw_fixture_inside_manual_2);

	sequence("place_screw_a_fixture").
		move(pos_screw_fixture_common);
		action(act_screw_path, act_screw_tcp, act_screw_fix1, pos_screw_fixture_common).reverseWith("place_screw_manual_1").neverReversible().
		check_start( screwing_succeded ).
		forceMode( fma_right , fma_deativate).
		wait(0.5).
		check(screw_1_on);
		io(screwdriver_activate, Switch::on).
		wait(0.3).
		wait(scrwing_finished).reverseWith("wait_for_screwing_backward").
		wait(0.3);
		io(screwdriver_activate,Switch::off).
		io(screwdriver_set_backwards, Switch::off);		forceMode( fma_deativate , fma_left).
		check_stop( screwing_succeded ).
		print("skip").reverseWith("screw_insert_nonReversible").
		forceMode( fma_deativate , fma_right);
		check(screw_1_inserted);
		print("skip").reverseWith("move_pos_screw_fixture_inside_1").
		move(pos_screw_fixture_at_1).
		move(pos_screw_fixture_common);

	sequence("place_screw_b_fixture").
		move(pos_screw_fixture_common);
		print("skip").
		action(act_screw_path, act_screw_tcp, act_screw_fix2, pos_screw_fixture_common).reverseWith("place_screw_manual_2").neverReversible().
		check_start( screwing_succeded ).
		forceMode( fma_right , fma_deativate).
		wait(0.5);
		check(screw_2_on);
		io(screwdriver_activate, Switch::on).
		wait(0.3).
		wait(scrwing_finished).reverseWith("wait_for_screwing_backward").
		wait(0.3);
		io(screwdriver_activate,Switch::off).
		io(screwdriver_set_backwards, Switch::off);		forceMode( fma_deativate , fma_left).
		check_stop( screwing_succeded ).
		print("skip").reverseWith("screw_insert_nonReversible").
		forceMode( fma_deativate , fma_right);
		check(screw_2_inserted);
		print("skip").reverseWith("move_pos_screw_fixture_inside_2").
		move(pos_screw_fixture_at_2).
		move(pos_screw_fixture_common);


// ******************************
// * MAIN SEQUENCE


	for(int i = 1; i <= 4; i++)
	{
		std::string n = std::to_string(i);
		sequence("seq_" + n);
			call("pick_bracket_back_" + n);
			call("place_bracket_back");
			call("zoned_lock");
			call("pick_bracket_front_" + n);
			call("place_bracket_front");
			call("from_bracket_to_screwdriver");
			call("pick_screwdriver");
			call("pick_screw_"+n+"a_table");
			call("place_screw_a_fixture");
			call("pick_screw_"+n+"b_table");
			call("place_screw_b_fixture");
			uncall("pick_screwdriver");
			uncall("from_bracket_to_screwdriver");
			uncall("zoned_lock");
			call("remove_finished_bracket");
	}

	sequence("main");
	for(int i = 1; i <= 5; i++)
	{
		call("seq_1");
		call("seq_2");
		call("seq_3");
		call("seq_4");
	}


}

} /* namespace dsl */
