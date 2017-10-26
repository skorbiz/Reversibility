/*
 * SequenceBuilder.cpp
 *
 *  Created on: Nov 17, 2014
 *      Author: josl
 */

#include "SequenceBuilder.hpp"

#include <memory>
#include <../src/model/basemodel/command/derivedCommands/intelligentMove/XMLRecordLoader.hpp>


namespace dsl {

SequenceBuilder::SequenceBuilder() :
	_lastElement(nullptr),
	_lastSequence(nullptr)
{
	std::string devname = "ur5";
	_nodeHnd = rw::common::Ptr<ros::NodeHandle>(new ros::NodeHandle);
	_pDigitalIO = std::shared_ptr<DigitalIOGeneralInterfaceProxy>(new DigitalIOGeneralInterfaceProxy(_nodeHnd, "Moxa")); //Moxa
	_pRobotiq = std::shared_ptr<RobotiqInterfaceProxy>(new RobotiqInterfaceProxy(_nodeHnd, devname));

	unsigned int urrt_port = 30003;
	unsigned int ur_port = 30001;
	std::string device_ip = "192.168.1.250";
	std::string callback_ip = "192.168.1.240";
	unsigned int numeric_callback_port = 33333;

	_urrtInterface = std::make_shared<rwhw::UniversalRobotsRTLogging>(universalrobots::CB31);
	_urInterface = std::make_shared<rwhw::URCallBackInterface>(universalrobots::CB31);

	std::cout << "Connecting to URRT interface" << std::endl;
	try
	{
		assert(_urrtInterface != nullptr);
		_urrtInterface->connect(device_ip, urrt_port);
	}
	catch (rw::common::Exception& exp)
	{
		std::cout << "Could not connect to urrt:" << exp.what() << std::endl;
	}
	std::cout << "Connected to URRT interface" << std::endl;
	std::cout << "Connecting to UR interface" << std::endl;
	try
	{
		assert(_urInterface != nullptr);
		_urInterface->connect(device_ip, ur_port);
	}
	catch (rw::common::Exception& exp)
	{
		std::cout << "Could not connect to urrt:" << exp.what() << std::endl;
	}
	std::cout << "Connected to UR interface" << std::endl;

	_urInterface->startCommunication(callback_ip, numeric_callback_port);
	_urrtInterface->start();
	std::cout << "Started commonication on UR interface" << std::endl;

	std::cout << "Opening Intelligent Move dataset" << std::endl;
	_imoveDatasetPath = dsl::common::getPath2dslOutputFolder() + "/iMoveSaves.xml";
	_iActionDatasetPath = dsl::common::getPath2dslOutputFolder() + "/iActionSaves.xml";
	dsl::intelligentmove::XMLRecordLoader loader;
	_imoveDataset = std::make_shared<dsl::intelligentmove::RecordDataset>(loader.load(_imoveDatasetPath));
	_iActionDataset = std::make_shared<dsl::intelligentmove::RecordDataset>(loader.load(_iActionDatasetPath));

}

SequenceBuilder::~SequenceBuilder()
{
/*	_urInterface->stopCommunication();
	_urrtInterface->stop();

	_nodeHnd->~NodeHandle();
	_pDigitalIO->~DigitalIOGeneralInterfaceProxy();
	_pRobotiq->~RobotiqInterfaceProxy();
*/
}

// ******************************************************************
// **** ACCESS FUNCTION
// ****

dsl::BaseProgram SequenceBuilder::getFinalProgram()
{
	std::shared_ptr<dsl::Sequence> seqMain = getSeq("main");
	seqMain->build();
	std::shared_ptr<dsl::Instruction> ins = seqMain->getInstructionStart();
	return dsl::BaseProgram(ins);
}


std::shared_ptr<Sequence> SequenceBuilder::getSeq(std::string name)
{
	return _sequences.getObject(name);
}


// ******************************************************************
// **** SEQUENCE LOGIC
// ****

SequenceBuilder & SequenceBuilder::sequence(std::string name)
{
	_lastSequence = std::make_shared<Sequence>(name);
	_lastElement = _lastSequence;
	_sequences.saveObject(name, _lastSequence);
	return * this;
}

SequenceBuilder & SequenceBuilder::macro(std::string name)
{
	sequence(name);
	return * this;
}

SequenceBuilder & SequenceBuilder::include(std::string name)
{
	nullptrErrorCheck(_lastSequence, "_lastSequence", "include");
	_lastElement = std::make_shared<dsl::ElementSequenceNested>(_sequences.getObject(name));
	_lastSequence->addElment(_lastElement);
	return * this;
}

SequenceBuilder & SequenceBuilder::nest(std::string name)
{
	include(name);
	return * this;
}

SequenceBuilder & SequenceBuilder::call(std::string name)
{
	include(name);
	return * this;
}



// ******************************************************************
// **** REVERSE LOGIC
// ****

SequenceBuilder & SequenceBuilder::callReverse(std::string sequenceName)
{
	nullptrErrorCheck(_lastSequence, "_lastSequence", "callReverse");
	_lastElement = std::make_shared<dsl::ElementSequenceNested>(_sequences.getObject(sequenceName),true, true);
	_lastSequence->addElment(_lastElement);
	return * this;
}

SequenceBuilder & SequenceBuilder::uncall(std::string sequenceName)
{
	return callReverse(sequenceName);
}

SequenceBuilder & SequenceBuilder::reverseWith(std::string sequenceName)
{
	nullptrErrorCheck(_lastElement, "_lastElement", "reverseWith");
	std::shared_ptr<Element> temp = std::make_shared<dsl::ElementSequenceNested>(_sequences.getObject(sequenceName),true, false);
	_lastElement->setReverseCounterpart(temp);
	_lastElement = temp;
	return * this;
}

SequenceBuilder & SequenceBuilder::inverses(std::string forwardSeq, std::string backwardSeq)
{
	nullptrErrorCheck(_lastSequence, "_lastSequence", "callReverse");
	nest(forwardSeq);
	neverReversible();
	reverseWith(backwardSeq);
	neverReversible();
	return * this;
}

SequenceBuilder & SequenceBuilder::neverReversible()
{
	nullptrErrorCheck(_lastElement, "_lastElement", "reverseProperty");
	_lastElement->setIsReversible(false);
	return * this;
}


//SequenceBuilder & SequenceBuilder::reverseProperty(const dsl::ReverseBehaviour reverseBehaviour)
//{
//	nullptrErrorCheck(_lastElement, "_lastElement", "reverseProperty");
//	dsl::ReverseBehaviour * arg = new dsl::ReverseBehaviour(reverseBehaviour);
//	dsl::ArgumentBuilder::arg(arg);
//	_lastElement->appendPermanentArgument(arg);
//	return * this;
//}
//
//SequenceBuilder & SequenceBuilder::reverseProperty(const dsl::Reversibility reversibility)
//{
//	nullptrErrorCheck(_lastElement, "_lastElement", "reverseProperty");
//	dsl::Reversibility * arg = new dsl::Reversibility(reversibility);
//	dsl::ArgumentBuilder::arg(arg);
//	_lastElement->appendPermanentArgument(arg);
//	return * this;
//}


// ******************************************************************
// **** COMMAND PRIMITIVES DEFAULTS LOGIC
// ****


SequenceBuilder & SequenceBuilder::cmd(std::shared_ptr<Element> element)
{
	nullptrErrorCheck(_lastSequence, "_lastSequence", "cmd");
	_lastElement = element;
	_lastSequence->addElment(_lastElement);
	return * this;
}

SequenceBuilder & SequenceBuilder::move(dsl::JointConfiguration q)
{
	move(q.get());
	return * this;
}

SequenceBuilder & SequenceBuilder::move(rw::math::Q q)
{
	std::shared_ptr<dsl::CommandExecutable> m = std::make_shared<Move>(_urrtInterface, _urInterface, q);
	std::shared_ptr<dsl::ElementCommand> element = std::make_shared<dsl::ElementCommand>(m);
	cmd(element);
	return * this;
}

SequenceBuilder & SequenceBuilder::move(Vector3D<> direction, dsl::common::WorkcellModel::frame frame)
{
	dsl::common::WorkcellModel wc;
	std::shared_ptr<dsl::CommandExecutable> mr = std::make_shared<MoveRelative>(direction,frame,wc,_urrtInterface, _urInterface);
	std::shared_ptr<dsl::ElementCommand> element = std::make_shared<dsl::ElementCommand>(mr);
	cmd(element);
	return * this;
}

SequenceBuilder & SequenceBuilder::intMove(rw::math::Q qTo)
{
	std::shared_ptr<dsl::CommandExecutable> im = std::make_shared<dsl::intelligentmove::IntelligentMoveModelInterface>(_urInterface, _urrtInterface, qTo,_imoveDataset,_imoveDatasetPath);
	std::shared_ptr<dsl::ElementCommand> element = std::make_shared<dsl::ElementCommand>(im);
	cmd(element);
	return * this;
}

SequenceBuilder & SequenceBuilder::wait(double time)
{
	std::shared_ptr<dsl::CommandExecutable> w = std::make_shared<Wait>(time);
	std::shared_ptr<dsl::ElementCommand> element = std::make_shared<dsl::ElementCommand>(w);
	cmd(element);
	return * this;
}

SequenceBuilder & SequenceBuilder::wait(dsl::ConditionInstant * condition)
{
	std::shared_ptr<dsl::WaitForCondition> w = std::make_shared<WaitForCondition>(condition);
	std::shared_ptr<dsl::ElementCommand> element = std::make_shared<dsl::ElementCommand>(w);
	cmd(element);
	return * this;
}

SequenceBuilder & SequenceBuilder::io(dsl::IOPorts ioPort, const dsl::Switch sw)
{
	std::shared_ptr<dsl::IOManipulation> io = std::make_shared<IOManipulation>(_pDigitalIO, ioPort.get(), sw.get());
	std::shared_ptr<dsl::ElementCommand> element = std::make_shared<dsl::ElementCommand>(io);
	cmd(element);
	return * this;
}

SequenceBuilder & SequenceBuilder::print(std::string text)
{
	std::shared_ptr<dsl::Print> p = std::make_shared<Print>(text);
	std::shared_ptr<dsl::ElementCommand> element = std::make_shared<dsl::ElementCommand>(p);
	cmd(element);
	return * this;
}

SequenceBuilder & SequenceBuilder::grasp(int angle)
{
	int force = 150;
	int speed = 250;
	std::shared_ptr<dsl::Grasp> g = std::make_shared<Grasp>(_pRobotiq,angle,force,speed);
	std::shared_ptr<dsl::ElementCommand> element = std::make_shared<dsl::ElementCommand>(g);
	cmd(element);
	return * this;
}

SequenceBuilder & SequenceBuilder::gripper_open()
{
	grasp(0);
	return * this;
}

SequenceBuilder & SequenceBuilder::gripper_close()
{
	grasp(255);
	return * this;
}

SequenceBuilder & SequenceBuilder::forceMode(dsl::ForceModeArgument fm_forward, dsl::ForceModeArgument fm_backward)
{
	std::shared_ptr<dsl::ForceMode> fm = std::make_shared<ForceMode>(_urrtInterface, _urInterface,fm_forward, fm_backward);
	std::shared_ptr<dsl::ElementCommand> element = std::make_shared<dsl::ElementCommand>(fm);
	cmd(element);
	return * this;
}

SequenceBuilder & SequenceBuilder::action(std::string path, std::string tcp, std::string fixture, dsl::JointConfiguration q)
{
	dsl::ActionFactory * af = new ActionFactory();
	af->addInterfaces(_urrtInterface, _urInterface);
	af->addFilePath(path);
	af->addFixtureFrame(fixture);
	af->addToolFrame(tcp);
	af->addArgument(q);
	std::shared_ptr<dsl::ElementCommand> element = std::make_shared<dsl::ElementCommand>(af->factoryMethode());
	cmd(element);
	return * this;
}


SequenceBuilder & SequenceBuilder::intAction(std::string filepathTrajectory, std::string frameTool, std::string frameFixture, dsl::JointConfiguration qInit)
{
	std::shared_ptr<dsl::intelligentmove::IntelligentActionModelInterface> intAct;
	intAct = std::make_shared<dsl::intelligentmove::IntelligentActionModelInterface>(
			_urInterface,
			_urrtInterface,
			_iActionDataset,
			_iActionDatasetPath,
			filepathTrajectory,
			frameTool,
			frameFixture,
			qInit.get());
	std::shared_ptr<dsl::ElementCommand> element = std::make_shared<dsl::ElementCommand>(intAct);
	cmd(element);
	return * this;
}



// ******************************************************************
// **** CONDITIONS AND ERROR LOGIC
// ****

SequenceBuilder & SequenceBuilder::check(dsl::ConditionInstant * condition)
{
	std::shared_ptr<ElementErrorCheck> element = std::make_shared<dsl::ElementErrorCheck>(condition);
	cmd(element);
	return * this;
}

SequenceBuilder & SequenceBuilder::check_start(dsl::ConditionMonitored * condition)
{
	std::shared_ptr<ElementErrorCheckMonitoredStart> element = std::make_shared<dsl::ElementErrorCheckMonitoredStart>(condition);
	cmd(element);
	return * this;
}

SequenceBuilder & SequenceBuilder::check_stop(dsl::ConditionMonitored * condition)
{
	std::shared_ptr<ElementErrorCheck> element = std::make_shared<dsl::ElementErrorCheck>(condition);
	cmd(element);
	return * this;
}

} /* namespace dsl */
