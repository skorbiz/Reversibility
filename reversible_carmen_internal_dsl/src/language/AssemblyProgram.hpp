/*
 * AssemblyProgram.hpp
 *
 *  Created on: Nov 17, 2014
 *      Author: josl
 */

#ifndef ASSEMBLYPROGRAM_HPP_
#define ASSEMBLYPROGRAM_HPP_

#include <cassert>

#include <../src/language/SequenceBuilder.hpp>

#include <../src/model/basemodel/BaseProgram.hpp>
#include <../src/model/basemodel/Instruction.hpp>
#include <../src/model/expandedmodel/Sequence.hpp>

namespace dsl
{

class AssemblyProgram : protected SequenceBuilder
{

public:
	AssemblyProgram();
	virtual ~AssemblyProgram();
	dsl::BaseProgram getProgram();

private:
	virtual void buildExpandedModel() = 0;

};

} /* namespace dsl */

#endif /* ASSEMBLYPROGRAM_HPP_ */
