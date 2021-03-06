#include <memory>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <d7cp_proxys/gripper/WSG25Proxy.h>
#include <d7cp_proxys/gripper/WSG25Gripper.h>
#include <d7cp_proxys/gripper/D7PCGripper.h>
#include <d7cp_proxys/InspectionSystem.h>
#include <d7cp_proxys/VisionSystem.h>
#include <d7cp_proxys/Anyfeeder.h>

#include <d7cp_proxys/robot_movement_interface/RosProxy.h>
#include <d7cp_proxys/robot_movement_interface/RobworkInterface.h>

#include "skill/spiral_skill.hpp"
#include "skill/wiggle_skill.h"
#include "skill/Compliant_pih_skill.h"
#include "D7cpWorkcell.h"
#include "DynamicGrasp.h"
#include "colors.hpp"
#include "Move.h"
#include "MoveLin.h"
#include "Move2Q.h"

#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rwlibs/calibration/xml/XmlCalibrationLoader.hpp>
#include <rwlibs/calibration/WorkCellCalibration.hpp>

class RosAutoSpin
{
public:
	bool _runSpinThread;
	boost::thread _spinThread;

	RosAutoSpin(int argc, char **argv)
	{
		ros::init(argc, argv, "reversible_dsl");
		ros::start();
		_runSpinThread = true;
		_spinThread = boost::thread(&RosAutoSpin::thread_func, this);
	}

	~RosAutoSpin()
	{
		_runSpinThread = false;
		_spinThread.join();
	}

	void thread_func()
	{
		while (_runSpinThread)
		{
			ros::spinOnce();
			boost::this_thread::sleep(boost::posix_time::milliseconds(1));
		}
	}
};




int main(int argc, char **argv)
{

	std::cout << "Strating Workcell initiation" << std::endl;
	std::shared_ptr<D7cpWorkcell> wcPtr = std::make_shared<D7cpWorkcell>();
	D7cpWorkcell& wc(*wcPtr.get());

	std::cout << "Workcell model initiated" << std::endl;


	RosAutoSpin thread(argc, argv);

	std::cout << "Starting WSG25" << std::endl;
	gripper::WSG25Proxy::Ptr wsgProxy = std::make_shared<gripper::WSG25Proxy>();
	gripper::WSG25Gripper::Ptr wsgInterface = std::make_shared<gripper::WSG25Gripper>(wsgProxy);
	gripper::D7PC_Gripper::Ptr wsg = std::make_shared<gripper::D7PC_Gripper>(wsgInterface);
	std::cout << "Starting Inspection system" << std::endl;
	std::shared_ptr<InspectionSystem> inspectionSystem = std::make_shared<InspectionSystem>();
	std::cout << "Starting vision system" << std::endl;
	std::shared_ptr<VisionSystem> visionSystem = std::make_shared<VisionSystem>();
	std::cout << "Starting anyfeeder" << std::endl;
	std::shared_ptr<Anyfeeder> anyfeeder = std::make_shared<Anyfeeder>();
	std::cout << "Starting robot_movement_interface::RosProxy" << std::endl;
	std::shared_ptr<robot_movement_interface::RosProxy> iiwaRosProxy = std::make_shared<robot_movement_interface::RosProxy>();
	std::cout << "Starting iiwa::RobworkInterface" << std::endl;
	std::shared_ptr<robot_movement_interface::RobworkInterface> iiwa = std::make_shared<robot_movement_interface::RobworkInterface>(iiwaRosProxy);
	std::cout << "Interfaces initiated" << std::endl;


	std::shared_ptr<Converter> converter = wc.converter;
	rw::kinematics::State state0 = wc.getDefaultState();
	rw::math::Transform3D<> baseTcamera = converter->frameTframe(wc.baseFrame, wc.cameraFrame, state0);
	Move2Q moveq(iiwa, wc.planner);
	MoveLin movel(iiwa, wc.plannerRedundency, wc.toolFrame);

	rw::kinematics::State state = wc.getDefaultState();
	rw::math::Transform3D<> toolTobject = inspectionSystem->get_toolTobject_ground_truth(InspectionSystem::ObjectType::OuterShell);
	rw::math::Transform3D<> baseTtoolAssemblyRetracted = converter->toBaseTtool(toolTobject, wc.outershellAssemblyRetractedFrame, state);
	rw::math::Transform3D<> baseTtoolAssembly = converter->toBaseTtool(toolTobject, wc.outershellAssemblyFrame, state);

	int i = 0;
	while(true)
	{
		wc.setQ(wc.qInspectOuterShell, state);
		rw::math::Transform3D<> baseTtoolR = converter->rotated_deg(0,i,0, baseTtoolAssembly);
		rw::math::Transform3D<> baseTtoolRetractedR = converter->rotated_deg(0,i,0, baseTtoolAssemblyRetracted);

		Move move5(iiwa, wc.planner, wc.wc, wc.device, wc.toolFrame);

		wc.setQ(iiwa->getQ(), state);
		if( !move5.plan(baseTtoolRetractedR, state ) )
		{
			std::cout << dsl::color::RED << "(air) could not rotated to: " << i << dsl::color::DEFAULT << std::endl;
			continue;
		}

		wc.setQ(move5.get_endQ(), state);
		if(!movel.is_in_reach(baseTtoolR, state))
		{
			std::cout << dsl::color::RED << "(lin) could not rotated to: " << i << dsl::color::DEFAULT << std::endl;
			continue;
		}

		std::cout << dsl::color::GREEN << "Rotated to: " << i << dsl::color::DEFAULT << std::endl;
//		move5.move_forward();
//		movel.move_forward(baseTtoolR);
//		wsg->open(wsg->outer_shell);
//		ros::Duration(3).sleep();
//		wsg->close(wsg->outer_shell);
//		movel.move_forward(baseTtoolRetractedR);
		i = (i+10)%360;
	}
}
