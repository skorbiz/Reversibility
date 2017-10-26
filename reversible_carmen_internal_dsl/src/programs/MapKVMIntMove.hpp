/*
 * MapKVMIntMove.hpp
 *
 *  Created on: May 12, 2016
 *      Author: josl
 */

#ifndef REVERSIBLE_DSL_SRC_PROGRAMS_MAPKVMINTMOVE_HPP_
#define REVERSIBLE_DSL_SRC_PROGRAMS_MAPKVMINTMOVE_HPP_

#include <iostream>
#include <../src/language/AssemblyProgram.hpp>
#include <../src/model/basemodel/argument/quantities/IOPorts.hpp>
#include <../src/model/basemodel/argument/quantities/JointConfiguration.hpp>
#include <../src/model/basemodel/argument/quantities/Switch.hpp>
#include <../src/model/basemodel/condition/conditions/DistanceComparison.hpp>
#include <../src/model/basemodel/condition/conditions/IoComparison.hpp>
#include <../src/model/basemodel/condition/conditions/PositionCartesianComparison.hpp>
#include <../src/model/basemodel/condition/conditions/PositionJointComparison.hpp>

namespace dsl
{

class MapKVMIntMove : public AssemblyProgram
{
public:
	MapKVMIntMove();
	virtual ~MapKVMIntMove();
	void buildExpandedModel();

};

} /* namespace dsl */

#endif /* REVERSIBLE_DSL_SRC_PROGRAMS_MAPKVMINTMOVE_HPP_ */
