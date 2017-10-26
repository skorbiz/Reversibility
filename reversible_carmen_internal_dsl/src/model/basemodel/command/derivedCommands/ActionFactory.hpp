/*
 * ActionFactory.hpp
 *
 *  Created on: Jan 21, 2015
 *      Author: josl
 */

#ifndef ACTIONFACTORY_HPP_
#define ACTIONFACTORY_HPP_

#include <cassert>

#include <ros/ros.h>
#include <ros/package.h>

#include <rw/RobWork.hpp>
#include <rw/math.hpp>
#include <rw/trajectory/Path.hpp>


#include <rwhw/universalrobots/UniversalRobotsRTLogging.hpp>
#include <rwhw/universalrobots/URCallBackInterface.hpp>
#include <rwhw/universalrobots/URDefinitions.hpp>

//#include <simulation/common/SceeneIKSolver.hpp>

#include <../src/model/basemodel/command/CommandExecutable.hpp>
#include <../src/model/basemodel/command/derivedCommands/Action.hpp>
#include <../src/model/basemodel/argument/quantities/JointConfiguration.hpp>


namespace dsl
{

class ActionFactory
{

public:
	ActionFactory();
	virtual ~ActionFactory();

	virtual std::shared_ptr<dsl::CommandExecutable> factoryMethode();

	void addInterfaces( std::shared_ptr<rwhw::UniversalRobotsRTLogging> urrt, std::shared_ptr<rwhw::URCallBackInterface> ur);
	void addFilePath(std::string std);
	void addToolFrame(std::string std);
	void addFixtureFrame(std::string std);
	void addArgument(dsl::JointConfiguration & arg);
	void addArgument(rw::math::Q q);

private:
	std::shared_ptr<rwhw::UniversalRobotsRTLogging> _urrt;
	std::shared_ptr<rwhw::URCallBackInterface> _ur;

	std::string _filePath;
	std::string _toolFrame;
	std::string _fixtureFrame;
	rw::math::Q _qStart;

};

} /* namespace dsl */

#endif /* ACTIONFACTORY_HPP_ */
