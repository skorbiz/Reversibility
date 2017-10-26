/*
 * ActionFactory.cpp
 *
 *  Created on: Jan 21, 2015
 *      Author: josl
 */

#include "ActionFactory.hpp"

#include "ActionIKSolver.hpp"
#include "../src/common/common.hpp"

namespace dsl {

ActionFactory::ActionFactory()
{
}

ActionFactory::~ActionFactory()
{
}

std::shared_ptr<dsl::CommandExecutable> ActionFactory::factoryMethode()
{

	std::string nameProjectRootPath = common::getPath2trunk();
//	std::string nameWokcellFilePath	= "/../../../scenes/CARMENscene_3.0/Scene_VOLA_jan15.wc.xml";
//	std::string nameWokcellFilePath	= "/../../../scenes/CARMENscene_3.0/Scene_KVM_feb15.wc.xml";
	std::string nameWokcellFilePath	= "/scenes/CARMENscene_3.0/Scene_DSL_apr15.wc.xml";
	std::string workcellFile = nameProjectRootPath + nameWokcellFilePath;

	ROS_INFO_STREAM("loading file: " << workcellFile );
	std::cout << _filePath << std::endl;

	assert(!_filePath.empty() && "Filepath in action was empty");
	assert(!_toolFrame.empty() && "Filepath in action was empty");
	assert(!_fixtureFrame.empty() && "Filepath in action was empty");

	std::cout << _filePath << std::endl;
	std::cout << _toolFrame << std::endl;
	std::cout << _fixtureFrame << std::endl;

	std::shared_ptr<dsl::ActionIKSolver> ikSolver = std::make_shared<ActionIKSolver>(
			workcellFile,
			"UR5",
			_toolFrame,
			_fixtureFrame);
//	std::cout << "test 5" << std::endl;

	ikSolver->setInitialQ(_qStart);
//	std::cout << "test 6" << std::endl;
	rw::trajectory::TimedQPath path = ikSolver->solve(_filePath);
//	std::cout << "test 6.5" << std::endl;
//	std::cout << "PATH::::::::::::::::" << path.size() << std::endl;
//	std::cout << "test 7" << std::endl;
	return std::make_shared<dsl::Action>(_urrt,_ur,path);
}

void ActionFactory::addInterfaces( std::shared_ptr<rwhw::UniversalRobotsRTLogging> urrt, std::shared_ptr<rwhw::URCallBackInterface> ur)
{
	_urrt = urrt;
	_ur = ur;
}

void ActionFactory::addFilePath(std::string std)
{
	_filePath = std;
}

void ActionFactory::addToolFrame(std::string std)
{
	_toolFrame = std;
}

void ActionFactory::addFixtureFrame(std::string std)
{
	_fixtureFrame = std;
}

void ActionFactory::addArgument(rw::math::Q q)
{
	_qStart = q;
}

void ActionFactory::addArgument(dsl::JointConfiguration & arg)
{
	_qStart = arg.get();
}


} /* namespace dsl */
