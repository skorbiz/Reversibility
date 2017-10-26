/*
 * MapKVM.hpp
 *
 *  Created on: May 19, 2015
 *      Author: josl
 */

#ifndef SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_LANGUAGE_PROGRAMS_MAPKVM_HPP_
#define SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_LANGUAGE_PROGRAMS_MAPKVM_HPP_

#include <iostream>
#include <../src/language/AssemblyProgram.hpp>
#include <../src/model/basemodel/argument/quantities/IOPorts.hpp>
#include <../src/model/basemodel/argument/quantities/JointConfiguration.hpp>
#include <../src/model/basemodel/argument/quantities/Switch.hpp>
#include <../src/model/basemodel/condition/conditions/DistanceComparison.hpp>
#include <../src/model/basemodel/condition/conditions/IoComparison.hpp>
#include <../src/model/basemodel/condition/conditions/PositionCartesianComparison.hpp>
#include <../src/model/basemodel/condition/conditions/PositionJointComparison.hpp>

namespace dsl {

class MapKVM  : public AssemblyProgram
{
public:
	MapKVM();
	virtual ~MapKVM();
	void buildExpandedModel();
};

} /* namespace dsl */

#endif /* SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_LANGUAGE_PROGRAMS_MAPKVM_HPP_ */
