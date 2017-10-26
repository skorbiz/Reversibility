/*
 * TestLanguage.hpp
 *
 *  Created on: Sep 7, 2015
 *      Author: josl
 */

#ifndef SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_PROGRAMS_TESTLANGUAGE_HPP_
#define SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_PROGRAMS_TESTLANGUAGE_HPP_

#include <../src/model/basemodel/controller/ControlUnit.hpp>
#include <../src/common/GraphPrint.hpp>
#include <../src/programs/MapKVM.hpp>
#include <../src/programs/MapKVMForceMode.hpp>
#include <../src/programs/MapTest.hpp>
#include <../src/programs/MapVola.hpp>
#include <../src/programs/MapKVMIntMove.hpp>

namespace dsl
{

class TestLanguage
{

public:
	TestLanguage(std::vector<std::string> args);
	virtual ~TestLanguage();

};

} /* namespace dsl */

#endif /* SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_PROGRAMS_TESTLANGUAGE_HPP_ */
