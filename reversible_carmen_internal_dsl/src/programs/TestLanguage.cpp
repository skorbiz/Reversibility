/*
 * TestLanguage.cpp
 *
 *  Created on: Sep 7, 2015
 *      Author: josl
 */

#include "TestLanguage.hpp"

namespace dsl {

TestLanguage::TestLanguage(std::vector<std::string> args)
{
	std::cout << "Language test: start" << std::endl;

//	dsl::MapTest map;
	dsl::MapKVM map;
//	dsl::MapVola map;
//	dsl::MapKVMForceMode map;
//	dsl::MapKVMIntMove map;

	dsl::BaseProgram bprogram = map.getProgram();

	std::cout << "\n Graph print: start" << std::endl;
	dsl::common::GraphPrint graphPrint(bprogram);

	while(true)
	{
 	std::cout << "\n Language test: Forward" << std::endl;
	dsl::ControlUnit cu(bprogram);
	cu.execute();
	}

	std::cout << "\n Language test: Reverse" << std::endl;
//	cu.execute_backwards();

	std::cout << "\n Language test: end" << std::endl;
}

TestLanguage::~TestLanguage()
{
}

} /* namespace dsl */
