/*
 * TestDistanceCondition.hpp
 *
 *  Created on: Jun 9, 2015
 *      Author: josl
 */

#ifndef SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_COMMON_TestDistanceCondition_HPP_
#define SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_COMMON_TestDistanceCondition_HPP_

#include <iostream>
#include <rw/common.hpp>
#include <RobotGeneralInterfaceProxy.hpp>
#include <RobotCartesianInterfaceProxy.hpp>
#include <../src/model/basemodel/condition/conditions/PositionJointComparison.hpp>
#include <../src/model/basemodel/condition/conditions/PositionCartesianComparison.hpp>

namespace dsl{
namespace common{

class TestDistanceCondition
{

public:
	TestDistanceCondition();
	virtual ~TestDistanceCondition();

	void positionTestJoin();
	void positionTestCartesian();

};

} /* namspace common */
} /* namespace dsl */

#endif /* SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_COMMON_TESTDISTANCECONDITION_HPP_ */
