/*
 * MapTest.hpp
 *
 *  Created on: Apr 30, 2015
 *      Author: josl
 */

#ifndef SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_LANGUAGE_PROGRAMS_MAPTEST_HPP_
#define SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_LANGUAGE_PROGRAMS_MAPTEST_HPP_

#include <iostream>
#include <../src/language/AssemblyProgram.hpp>
#include <../src/model/basemodel/argument/quantities/IOPorts.hpp>
#include <../src/model/basemodel/argument/quantities/JointConfiguration.hpp>
#include <../src/model/basemodel/argument/quantities/Switch.hpp>
#include <../src/model/basemodel/condition/conditions/DistanceComparison.hpp>
#include <../src/model/basemodel/condition/conditions/IoComparison.hpp>
#include <../src/model/basemodel/condition/conditions/PositionCartesianComparison.hpp>
#include <../src/model/basemodel/condition/conditions/PositionJointComparison.hpp>
#include <../src/model/basemodel/condition/conditions/IoComparisonMonitored.hpp>

namespace dsl
{

class MapTest : public AssemblyProgram
{

public:
	MapTest();
	virtual ~MapTest();

	void buildExpandedModel();

};

} /* namespace dsl */

#endif /* SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_LANGUAGE_PROGRAMS_MAPTEST_HPP_ */
