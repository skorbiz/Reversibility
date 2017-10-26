/*
 * TestNikolajInterface.cpp
 *
 *  Created on: Jul 4, 2016
 *      Author: josl
 */

#include "TestNikolajInterface.hpp"
#include <../src/common/common.hpp>
#include <../src/language/xml/XMLloader.hpp>
#include <../src/model/basemodel/controller/ControlUnitThreaded.hpp>

TestNikolajInterface::TestNikolajInterface()
{

	std::cout << "Started nikolaj test" << std::endl;

	std::string filepath = dsl::common::getPath2sduMagic() + "/src/programs/MapKVM.xml";
	std::cout << "Filepath: " << filepath;

	dsl::XMLloader loader;
	dsl::BaseProgram program = loader.load(filepath);
	std::cout << "program loaded" << std::endl;


	dsl::ControlUnitThreaded cu(program);
	cu.startExecutionForward();

	while(true)
		if(cu.getInstructionCurrent() != nullptr)
		{
			std::cout << cu.getInstructionCurrent()->getID() << std::endl;
			ros::Duration(0.5).sleep();
		}
	std::cout << "Program executed" << std::endl;
}

TestNikolajInterface::~TestNikolajInterface()
{

}
