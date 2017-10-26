/*
 * TestLanguageXML.cpp
 *
 *  Created on: Oct 22, 2015
 *      Author: josl
 */

#include "TestLanguageXML.hpp"

namespace dsl {

TestLanguageXML::TestLanguageXML()
{
	std::cout << "XML Language test: start" << std::endl;
	dsl::XMLloader xml;
	dsl::BaseProgram map(xml.load());
	dsl::ControlUnit cu(map);

	std::cout << "\n Graph print: start" << std::endl;
	dsl::common::GraphPrint graphPrint(map);

//	std::cout << "\n Language test: Forward" << std::endl;
//	cu.execute();
//
////		std::cout << "\n Language test: Reverse" << std::endl;
////		cu.execute_backwards();
//
//	std::cout << "\n Language test: end" << std::endl;

}

TestLanguageXML::~TestLanguageXML()
{
}

} /* namespace dsl */
